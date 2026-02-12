# LIMAP 工程架构与 Hypersim Triangulation 全流程笔记

## 1. 项目定位

LIMAP 是一个“线特征驱动”的视觉几何工具箱，核心目标是：

- 在多视图图像中重建 3D 线地图（line mapping）
- 基于点+线联合约束做定位（hybrid localization）
- 为 2D/3D 线相关几何操作提供可复用的 API

其实现是典型的 **Python 编排 + C++ 计算内核（pybind11）** 混合架构。

---

## 2. 工程总体架构（分层）

### 2.1 入口/应用层

- `runners/`：按数据集和任务组织的可执行脚本（Hypersim、ScanNet、ETH3D、COLMAP、7Scenes、Cambridge、InLoc 等）
- `scripts/`：下载数据、评测、格式转换等辅助脚本
- `visualize_3d_lines.py`：重建结果可视化入口

### 2.2 流程编排层（Python）

- `src/limap/runners/`：主流程接口
- 代表接口：
  - `line_triangulation`（RGB-only 三角化重建）
  - `line_fitnmerge`（有深度时的 fit & merge）
  - `hybrid_localization`（点线联合定位）
- `src/limap/util/config.py`：YAML 读取、配置继承、命令行动态覆写
- `src/limap/util/io.py`：中间结果与最终结果读写

### 2.3 算法模块层（Python API + C++绑定）

- `src/limap/base`：几何基础数据结构（Camera / CameraPose / CameraImage / ImageCollection / Line2d / Line3d / LineTrack）
- `src/limap/line2d`：2D 线检测/描述/匹配统一接口与注册器
- `src/limap/pointsfm`：COLMAP/hloc 互操作、邻居与场景范围计算
- `src/limap/triangulation`：线三角化核心（含 `GlobalLineTriangulator`）
- `src/limap/merging`：轨迹融合与过滤
- `src/limap/fitting`：深度/点云约束下的 3D 线拟合
- `src/limap/optimize`：BA、全局关联、线优化、联合定位优化
- `src/limap/estimators`：绝对位姿估计（点线混合 RANSAC）
- `src/limap/vplib`：消失点检测封装（JLinkage / Progressive-X）
- `src/limap/structures`：点线二部图等结构
- `src/limap/undistortion`：去畸变
- `src/limap/visualize`：Open3D 可视化封装
- `src/limap/evaluation`：评估指标

### 2.4 原生内核层（C++）

- `src/limap/internal/bindings.cc` 统一创建 `_limap` Python 扩展模块
- 各子模块 C++ 内核通过 pybind 暴露为 `_limap._base/_triangulation/_merging/...`
- `CMakeLists.txt` + `scikit-build-core`（`pyproject.toml`）负责构建

### 2.5 配置与文档层

- `cfgs/`：不同任务/数据集配置（triangulation、fitnmerge、localization、eval 等）
- `docs/`：教程与 API 文档
- `tests/`：基础和第三方依赖相关测试

---

## 3. 模块职责（按“数据流”理解）

### 3.1 输入与相机组织

- 数据集 loader（如 `runners/hypersim/loader.py`）把原始文件转换为 `ImageCollection`
- `ImageCollection` 是后续全部流程的统一输入载体（相机内外参 + 图像路径 + 图像 id）

### 3.2 辅助元信息（SfM）

- `limap.runners.compute_sfminfos` 通过 `limap.pointsfm` 调用 COLMAP/hloc：
  - 计算每张图的视觉邻居（neighbors）
  - 估计场景 3D robust ranges

### 3.3 2D 线前端

- `limap.runners.compute_2d_segs`：检测 2D 线段（可选描述子）
- `limap.runners.compute_matches` / `compute_exhaustive_matches`：线匹配
- `line2d/register_*`：用配置选择具体 detector/extractor/matcher

### 3.4 多视图线三角化

- `GlobalLineTriangulator` 执行候选生成、打分、连边、聚类、轨迹聚合
- 支持的候选来源：
  - 纯代数线三角化
  - one-point / many-points（可借助 point SfM 的点线二部图）
  - VP 方向先验

### 3.5 后处理与优化

- `merging`：重投影过滤、remerge、敏感性过滤、重叠过滤
- `optimize`：线 BA / 混合 BA（按配置开关）

### 3.6 结果落盘与可视化

- `save_folder_linetracks_with_info`：保存 track 文件 + 配置 + imagecols + 2D 线
- `save_obj`：导出 OBJ 线模型
- `visualize_3d_lines.py`：离线可视化

---

## 4. 重点：`python runners/hypersim/triangulation.py --output_dir outputs/quickstart_triangulation` 完整流程

下面按真实调用链展开。

## 4.1 入口脚本阶段

文件：`runners/hypersim/triangulation.py`

1. `parse_config()`
- 默认配置链：
  - 主配置：`cfgs/triangulation/hypersim.yaml`
  - 默认底座：`cfgs/triangulation/default.yaml`
- `parse_known_args()` 只显式解析少量参数（`config_file/default_config_file/npyfolder`）
- 其他参数（比如你传入的 `--output_dir`）进入 `unknown`，由 `limap.util.config.update_config` 按 key-path 写回配置
- 快捷参数映射：`-nv -> --n_visible_views`, `-nn -> --n_neighbors`, `-sid -> --scene_id`

2. `main()`
- 构建数据集对象：`dataset = Hypersim(cfg["data_dir"])`
- 调用：`run_scene_hypersim(cfg, dataset, cfg["scene_id"], cam_id=cfg["cam_id"])`

---

## 4.2 Hypersim 数据读取阶段

文件：`runners/hypersim/loader.py` + `runners/hypersim/Hypersim.py`

`read_scene_hypersim(..., load_depth=False)` 关键动作：

1. `dataset.set_scene_id(scene_id)`：定位场景目录并读取 `meters_per_asset_unit`
2. `dataset.set_max_dim(cfg["max_image_dim"])`：按最大边限制缩放相机分辨率与内参
3. 构建候选帧 id：`range(0, input_n_views, input_stride)`，再 `filter_index_list` 过滤实际存在图像
4. 读取相机轨迹（`camera_keyframe_positions/orientations.hdf5`），并做坐标系变换（含 `R180x`）
5. 构建：
  - `base.Camera("SIMPLE_PINHOLE", K, ...)`
  - 每帧一个 `base.CameraImage`
  - 最终 `base.ImageCollection`
6. 因为 `load_depth=False`，此路径不加载深度

输出：`imagecols`

---

## 4.3 主流程 `limap.runners.line_triangulation`（A-F 六阶段）

文件：`src/limap/runners/line_triangulation.py`

### A. setup + SfM 元信息

1. `runners.setup(cfg)`
- 设置 `dir_save`（这里就是 `outputs/quickstart_triangulation`）
- 设置 `dir_load`

2. 去畸变检查
- 若 `imagecols` 非 undistorted，则跑 `undistort_images`
- Hypersim 当前相机模型通常是 `SIMPLE_PINHOLE`，一般可视为已去畸变

3. 先保存基础信息
- `image_list.txt`
- `imagecols.npy`

4. `compute_sfminfos(cfg, imagecols)`
- 因为 `neighbors` 未显式传入，会触发 COLMAP 已知位姿三角化
- 调用链：
  - `pointsfm.run_colmap_sfm_with_known_poses`
  - 内部 `run_hloc_matches` 做点特征提取与匹配
  - `pycolmap.triangulate_points` 做点云三角化
- 基于点云模型计算：
  - `neighbors`（每图前 `n_neighbors` 个视觉邻居）
  - `ranges`（场景稳健空间范围）
- 写出 `metainfos.txt`

### B. 2D 线段检测

1. `compute_descinfo` 判定
- 你的 quickstart 组合里：
  - `triangulation.use_exhaustive_matcher = True`
  - `line2d.compute_descinfo = False`
- 结果：**只做检测，不做描述子提取**

2. `compute_2d_segs`
- detector 使用 `lsd`（来自 `hypersim.yaml`）
- 输出目录：`line_detections/lsd/segments`
- 若配置 `line2d.visualize=True`（默认 true），还会写检测可视化图

### C. 线匹配阶段

- 仅当 `use_exhaustive_matcher=False` 才会跑 `compute_matches`
- 你的 quickstart 为 `True`，因此此阶段跳过
- 后续三角化对每对邻居图采用“线段全配对穷举”

### D. 多视图三角化核心

1. 初始化
- `Triangulator = triangulation.GlobalLineTriangulator(cfg["triangulation"])`
- `Triangulator.SetRanges(ranges)`
- `Triangulator.Init(all_2d_lines, imagecols)`

2. 可选分支（quickstart 默认都关闭）
- `use_vp=False`：不注入 VP
- `use_pointsfm.enable=False`：不注入点线二部图/3D点

3. 主循环
- 对每个 `img_id` 调 `TriangulateImageExhaustiveMatch(img_id, neighbors[img_id])`

4. C++ 内核中的关键逻辑（`base_line_triangulator.cc` + `global_line_triangulator.cc`）
- 对图像 i 的每条 2D 线，与每个邻居图 j 的每条 2D 线组成候选
- 候选先过几何筛选：
  - 线长阈值
  - 退化角度检查（ray-plane angle）
  - epipolar IoU 阈值
  - 敏感度阈值
- 通过后做三角化（默认 algebraic line triangulation）并计算不确定度
- `ScoringCallback` 对候选按跨视图支持打分
- 保留高分连接，构图聚类（默认 `merging_strategy=greedy`）
- 聚类后聚合成 `LineTrack`

5. 过滤
- `filter_tracks_by_reprojection`
- 可选 `remerge`（quickstart 默认启用）
- `filter_tracks_by_sensitivity`
- `filter_tracks_by_overlap`
- 可见视角数阈值筛选（`n_visible_views`）

### E. 几何优化（BA）

- quickstart 覆写了 `refinement.disable=True`，所以 **本次不跑 BA**

### F. 结果保存与可视化

1. 保存
- `alltracks.txt`（可见帧数>=4的track摘要）
- `finaltracks/`（逐 track 文本 + 附加信息）
- `triangulated_lines_nv{n_visible_views}.obj`

2. 若 `visualize=True`（默认 true）
- 会先 `input("Press Enter...")` 等待交互
- 然后 Open3D 弹窗可视化

---

## 4.4 该命令运行后的典型产物结构

以 `--output_dir outputs/quickstart_triangulation` 为例：

```text
outputs/quickstart_triangulation/
  image_list.txt
  imagecols.npy
  metainfos.txt
  colmap_outputs/
    db.db
    images/
    sparse/
      ...
  line_detections/
    lsd/
      segments/
      visualize/           # 若 line2d.visualize=True
  alltracks.txt
  triangulated_lines_nv4.obj
  finaltracks/
    track_0.txt
    track_1.txt
    ...
    config.npy
    imagecols.npy
    all_2d_segs.npy
  undistorted_images/      # 仅在输入需去畸变时出现
```

说明：

- 因为 `use_exhaustive_matcher=True`，一般不会生成 `line_matchings/...` 目录
- 因为 `refinement.disable=True`，不会有 BA 后的新 track 输出分支

---

## 4.5 一句话时序图

`triangulation.py`（解析配置）
-> `Hypersim loader`（构建 ImageCollection）
-> `line_triangulation`:
`setup`
-> `compute_sfminfos(COLMAP+hloc)`
-> `compute_2d_segs(LSD)`
-> `GlobalLineTriangulator`（穷举邻居线对 + 三角化 + 打分聚类）
-> `filters/remerge`
-> `save finaltracks + obj`
-> `Open3D visualize(可选)`

---

## 5. 这个 quickstart 配置的工程性取舍

- 优先可跑性：`use_cuda=False`、`lsd`、`compute_descinfo=False`
- 用穷举线对代替描述子匹配：`use_exhaustive_matcher=True`
- 默认关闭 BA：`refinement.disable=True`

因此它更像“最小闭环演示路径”：

- 保留完整主流程
- 降低外部模型依赖和配置复杂度
- 在 100 张示例图上快速出结果

---

## 6. line detector / extractor / matcher 全量实现（重点）

这一节按工程里的真实实现（`src/limap/line2d/*`）整理。  
先说结论：LIMAP 的 line2d 前端是“注册器 + 多实现后端”的插件化架构，核心入口是：

- detector 注册：`src/limap/line2d/register_detector.py`
- extractor 注册：`src/limap/line2d/register_detector.py`
- matcher 注册：`src/limap/line2d/register_matcher.py`

另外一个关键点：如果启用 `triangulation.use_exhaustive_matcher=True`（quickstart 就是这样），那么三角化阶段会跳过 line matcher，直接对邻居图中的线段做穷举配对；此时 extractor/matcher 影响会显著降低，detector 仍是核心。

### 6.1 Detector 实现（5种）

1. `lsd`（`src/limap/line2d/LSD/lsd.py`）
- 原理：经典 LSD（Line Segment Detector），基于梯度与统计显著性，非深度学习。
- 实现：直接调用 `pytlsd.lsd(img)`。
- 工程效果：CPU 友好、速度快、依赖最轻，适合快速跑通和大规模批处理；在弱纹理/复杂光照场景下稳定性通常不如学习方法。

2. `sold2`（`src/limap/line2d/SOLD2/sold2.py` + `sold2_wrapper.py`）
- 原理：SOLD2 端到端线段检测+描述框架；匹配侧采用 Needleman-Wunsch 思路做线上采样点序列匹配。
- 实现细节：检测输出线段、descriptor、heatmap，线得分来自沿线 heatmap saliency 聚合。
- 工程效果：在复杂场景通常比传统方法更稳健；但模型更重、推理更慢，依赖 CUDA/权重下载。

3. `deeplsd`（`src/limap/line2d/DeepLSD/deeplsd.py`）
- 原理：DeepLSD 学习式线检测（结合深度预测与LSD风格几何约束）。
- 实现细节：`DeepLSD(conf)` 推理，封装里把线长当作 score。
- 工程效果：泛化与鲁棒性较强；比 LSD 更重。其封装支持 `cuda` 不可用时回落 CPU（速度会明显下降）。

4. `hawpv3`（`src/limap/line2d/HAWPv3/hawp.py`）
- 原理：HAWP 系列的 wireframe 解析思路，联合建模 junction 与 line。
- 实现细节：读取 HAWPv3 权重，按分数阈值过滤线段。
- 工程效果：对结构化场景（室内、建筑）通常有优势；依赖额外安装（`misc/install/hawpv3.md`），封装里默认走 CUDA。

5. `tp_lsd`（`src/limap/line2d/TP_LSD/tp_lsd.py`）
- 原理：TP-LSD 学习式线检测，带针对亮度/纹理的预处理增强。
- 实现细节：先做 HSV-V 通道增强与平滑，再喂入 TP-LSD 网络；score 用线长近似。
- 工程效果：复杂光照/纹理下稳定性较好；安装门槛较高（`misc/install/tp_lsd.md`，对编译器版本也有限制），封装里默认走 CUDA。

### 6.2 Extractor 实现（7种）

1. `sold2`
- 文件：`src/limap/line2d/SOLD2/sold2.py`
- 原理：复用 SOLD2 的 descriptor 特征图，在每条线采样并聚合描述子。
- 效果：与 SOLD2 matcher 配套时一致性最好；速度/显存开销中高。

2. `lbd`
- 文件：`src/limap/line2d/LBD/extractor.py`
- 原理：传统 LBD（Line Band Descriptor），多尺度金字塔 + 线带描述。
- 效果：CPU 友好、描述紧凑、工程稳定；区分性通常弱于新型学习描述子。

3. `l2d2`
- 文件：`src/limap/line2d/L2D2/extractor.py`
- 原理：围绕每条线裁 48x32 归一化 patch，经 CNN 输出 128D 描述子。
- 效果：比传统描述子有更好判别力；但 patch 裁剪+网络前向带来额外耗时。

4. `linetr`
- 文件：`src/limap/line2d/LineTR/extractor.py` + `line_transformer.py`
- 原理：SuperPoint 密集特征 + LineTR Transformer，把子线 token 聚合成线描述。
- 效果：判别力强、适合中高难匹配；代价是显存/时延更高。

5. `superpoint_endpoints`
- 文件：`src/limap/line2d/endpoints/extractor.py`
- 原理：只在线段两个端点采样 SuperPoint 描述子，再在 matcher 侧做线级组合。
- 效果：实现简单、速度快，适合作为高效基线；对端点定位质量较敏感。

6. `wireframe`
- 文件：`src/limap/line2d/GlueStick/extractor.py`
- 原理：SuperPoint + `lines_to_wireframe`，构建 junction/line 结构化输入（线端点索引、junction描述等）。
- 效果：为 GlueStick 提供图结构上下文，匹配鲁棒性较强；流程更重。

7. `dense_naive`
- 文件：`src/limap/line2d/dense/extractor.py`
- 原理：不提显式线描述子，只保存图像、线段和分数，供 dense matcher 做几何一致性匹配。
- 效果：本身开销低，但依赖后续 dense_roma 的计算。

### 6.3 Matcher 实现（8种）

1. `sold2`
- 文件：`src/limap/line2d/SOLD2/sold2.py` + `SOLD2/model/line_matching.py`
- 原理：在线段上采样点并用 Needleman-Wunsch/序列对齐式匹配，再可选互检。
- 效果：对部分遮挡、局部错位更有韧性；计算量中高。

2. `lbd`
- 文件：`src/limap/line2d/LBD/matcher.py`
- 原理：`pytlbd.lbd_matching_multiscale` 做多尺度 LBD 匹配。
- 效果：CPU 上效率高、稳定；能力上偏传统。

3. `l2d2`
- 文件：`src/limap/line2d/L2D2/matcher.py`
- 原理：描述子相似度矩阵 + mutual nearest neighbor（或 top-k）。
- 效果：实现简洁、速度可控；效果主要受提取器质量影响。

4. `linetr`
- 文件：`src/limap/line2d/LineTR/matcher.py`
- 原理：先算 subline 距离，再映射到 keyline 距离矩阵，阈值+互检匹配。
- 效果：匹配精度较好，尤其在中高难视角变化下；代价偏高。

5. `nn_endpoints`
- 文件：`src/limap/line2d/endpoints/matcher.py`
- 原理：端点描述子相似度 -> 线级组合分数 -> Sinkhorn OT -> 线匹配。
- 效果：速度较快、配置简单；通常是高效方案首选。

6. `superglue_endpoints`
- 文件：`src/limap/line2d/endpoints/matcher.py`
- 原理：先跑 SuperGlue 端点匹配，再聚合到线级并做 Sinkhorn。
- 效果：比纯 NN 端点更鲁棒，尤其在歧义重复纹理下；耗时更高。

7. `gluestick`
- 文件：`src/limap/line2d/GlueStick/matcher.py`
- 原理：利用 junction + line + connectivity 的图匹配网络（GlueStick）。
- 效果：对结构化场景和复杂几何关系表现通常更稳；但依赖和算力开销较高。

8. `dense_roma`
- 文件：`src/limap/line2d/dense/matcher.py` + `dense_matcher/roma.py`
- 原理：先由 RoMa 估计像素级 dense warp/certainty；再沿线采样点，按几何距离与重叠率筛选对应。
- 效果：对大视角变化和重复纹理有优势；计算重，且需额外安装 `romatch`（`misc/install/roma.md`）。

### 6.4 兼容配对关系（工程里必须对齐）

下面这些组合是代码里明确约束/默认配套的：

- `sold2 extractor` ↔ `sold2 matcher`
- `lbd extractor` ↔ `lbd matcher`
- `l2d2 extractor` ↔ `l2d2 matcher`
- `linetr extractor` ↔ `linetr matcher`
- `superpoint_endpoints extractor` ↔ `nn_endpoints` / `superglue_endpoints`
- `wireframe extractor` ↔ `gluestick matcher`
- `dense_naive extractor` ↔ `dense_roma matcher`

### 6.5 选型建议（结合现有配置）

1. 极致快速/快速验证：
- 组合：`lsd + superpoint_endpoints + nn_endpoints`
- 对应：`cfgs/triangulation/default_fast.yaml`
- 特点：速度优先，README 已明确会有性能下降。

2. 纯 CPU 场景：
- 组合：`lsd + lbd + lbd`
- 对应：`cfgs/triangulation/default_cpu.yaml`
- 特点：无重型深度模型依赖，部署门槛低。

3. 默认鲁棒配置：
- 组合：`deeplsd + wireframe + gluestick`
- 对应：`cfgs/triangulation/default.yaml`
- 特点：精度/鲁棒性优先，资源需求高。

4. quickstart 当前命令：
- 组合本质：`lsd` 检测 + 三角化阶段穷举线对（`use_exhaustive_matcher=True`）
- 影响：extractor/matcher 并非主瓶颈，detector 和后端三角化参数更关键。

### 6.6 关于“效果”的边界说明

仓库内没有统一、同协议的 detector/extractor/matcher 全量 benchmark 表。  
上面的“效果”是基于实现机制与默认配置定位给出的工程结论，适合做第一轮选型；若要做严谨对比，建议在同一数据集和同一阈值协议下跑一轮 ablation（可从 `cfgs/triangulation/default*.yaml` 派生配置）。

---

## 7. 误检与误匹配抑制机制（全流程）

你提到的“纹理密集导致重复检测、误匹配”在 LIMAP 里不是靠单一模块解决，而是靠多级闸门串联。按执行顺序看：

### 7.1 2D 检测阶段：先压重复与低质量线

1. 检测后可选线段合并（去重复）
- 开关：`line2d.do_merge_lines`
- 位置：`src/limap/line2d/base_detector.py` -> `merge_lines`
- 算法：按正交距离和重叠关系聚类后合并（`src/limap/line2d/line_utils/merge_lines.py`）。

2. 长度优先截断（抑制短碎线）
- 参数：`line2d.max_num_2d_segs`
- 位置：`BaseDetector.take_longest_k`
- 作用：按长度保留 top-K，短纹理碎线会被大量清理。

3. 各 detector 自带阈值
- 如 HAWPv3 里的 `thresh`、SOLD2/TP-LSD/DeepLSD 各自检测配置。
- 作用：先在方法内部清掉低置信线段。

### 7.2 匹配前约束：减少“该匹配但不该比”的组合

1. 邻居图裁剪（不是所有图都互配）
- 参数：`n_neighbors`、`sfm.min_triangulation_angle`、`sfm.neighbor_type`
- 位置：`src/limap/pointsfm/functions.py`
- 作用：只在 SfM 共视邻居间匹配，大幅减少远距误配。

2. 点特征匹配含几何验证（用于 SfM 邻居构建）
- 位置：`src/limap/pointsfm/colmap_sfm.py` 中 `triangulation.estimation_and_geometric_verification`
- 作用：把几何不一致的点匹配先剔除，邻居关系更干净。

### 7.3 线匹配阶段：匹配器内部抑制误配

1. Mutual NN / Cross-check
- L2D2、LineTR、SOLD2 等都包含互检或等价机制。

2. Sinkhorn/OT 约束
- `nn_endpoints`、`superglue_endpoints` 使用 Sinkhorn 做线级分配，不是纯贪心最近邻。

3. Top-k 控制与阈值
- 参数：`line2d.matcher.topk`
- 作用：限制每条线候选数，防止一对多爆炸。

4. Dense matcher 的双向一致性与重叠门槛
- `dense_roma` 在 `src/limap/line2d/dense/matcher.py` 同时检查双向 warp、一致重叠、像素阈值。

### 7.4 三角化阶段：几何闸门过滤假匹配

位置：`src/limap/triangulation/base_line_triangulator.cc`

核心闸门：
- `min_length_2d`：过短线直接跳过
- `line_tri_angle_threshold`：退化视角（近平面）剔除
- `IoU_threshold`：弱极线一致性剔除
- `sensitivity_threshold`：不稳定三角化剔除
- `SetRanges(ranges)`：超出场景范围的候选剔除

可选增强：
- `use_vp`：用 VP 方向先验压制歧义
- `use_pointsfm`：用点-线二部图/3D点约束线候选

### 7.5 全局打分与轨迹建图：再压一次错误连接

位置：`src/limap/triangulation/global_line_triangulator.cc`

1. 跨视图支持打分
- 对每条候选线，用 `min(score3d, score2d)` 累积跨图支持，单图偶然误配难以高分通过。

2. 候选连接上限与分数阈值
- `fullscore_th`、`max_valid_conns`

3. 外边数量迭代剪枝
- `min_num_outer_edges` + 迭代传播删除，孤立/弱连接节点会被连锁清理。

4. 2D/3D Linker 联合规则
- 位置：`src/limap/base/line_linker.cc`
- 规则：角度、重叠、smart-angle、垂距、innerseg、scale-inv 等阈值联合判定。

### 7.6 轨迹后处理：重投影与重叠一致性过滤

位置：`src/limap/runners/line_triangulation.py` + `src/limap/merging/merging_utils.cc`

1. `filter_tracks_by_reprojection`
- 阈值：`th_angular_2d`、`th_perp_2d`
- 删除不支持当前 3D 线的 2D 观测，再重聚合线段。

2. `remerge` + 再过滤
- 迭代重合并后再次重投影过滤，减少重复 track 与错误合并。

3. `filter_tracks_by_sensitivity`
- 阈值：`th_sv_angular_3d`、`th_sv_num_supports`
- 去掉几何不稳定 track。

4. `filter_tracks_by_overlap`
- 阈值：`th_overlap`、`th_overlap_num_supports`
- 去掉投影重叠不足的伪 track。

5. 最终可见视角门槛
- 参数：`n_visible_views`
- 作用：抑制只被少量偶然匹配支撑的线。

### 7.7 BA/优化阶段：鲁棒损失与弱约束冻结

位置：`src/limap/optimize/line_refinement/refinement_config.h`、`src/limap/optimize/hybrid_bundle_adjustment/hybrid_bundle_adjustment.cc`

1. 鲁棒损失
- 线几何默认 `CauchyLoss(0.25)`，VP/热图/特征项也有独立 loss。
- 作用：降低离群观测对优化结果的牵引。

2. 支持度不足不优化
- 参数：`min_num_images`
- 作用：观测不足的线可保持常量，避免被噪声“优化坏”。

3. 相机/内参可设为常量
- `constant_intrinsics`、`constant_pose` 等
- 作用：避免弱约束场景中过拟合。

4. 输出线段仍做 outlier-trim 聚合
- `num_outliers_aggregator` 用于截断端点投影极值，减小少量错误支持的影响。

### 7.8 定位分支（补充）：额外误匹配抑制

位置：`src/limap/optimize/hybrid_localization/functions.py` + `estimators`

- `epipolar_filter` / `IoU_threshold`：先过极线一致性
- `reprojection_filter_matches_2to3`：再按重投影距离和角度筛选
- 最后 `RANSAC / Hybrid RANSAC`（点线双阈值）估计位姿，系统性剔除离群匹配

### 7.9 Fit&Merge 分支（补充）：每条线先过 3D RANSAC

位置：`src/limap/fitting/fitting.py`

- 深度/点云拟合时对每条 2D 线做 3D 点集 RANSAC
- 关键参数：`ransac_th`、`min_percentage_inliers`
- 低内点率直接丢弃，不进入后续 track 构建

### 7.10 你这个场景（纹理多、重复线多）的优先调参顺序

1. 先减重复检测
- 打开/加强 `line2d.do_merge_lines`
- 降低 `line2d.max_num_2d_segs`
- 提高 detector 的置信阈值（对应方法配置）

2. 再收紧三角化几何门槛
- 提高 `triangulation.IoU_threshold`
- 提高 `triangulation.line_tri_angle_threshold`
- 降低 `triangulation.sensitivity_threshold`（更严格）

3. 最后收紧 track 过滤
- 降低 `filtering2d.th_angular_2d`、`filtering2d.th_perp_2d`
- 提高 `n_visible_views`

这三步通常比直接“换一个 matcher”更快见效。
