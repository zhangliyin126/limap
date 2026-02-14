# LIMAP 工程架构与 Hypersim Triangulation 全流程笔记（重构版）

## 0. 文档说明

这份文档由原始“问答式记录”重排为 4 个主线：

1. 先看工程基础（项目目标、分层、数据流）
2. 再看 quickstart 真实调用链（从命令到产物）
3. 然后看 line2d 组件地图（detector/extractor/matcher）
4. 最后看误检抑制机制与调参顺序

---

## 1. 工程基础：项目定位、分层、数据流

### 1.1 项目定位

LIMAP 是一个“线特征驱动”的视觉几何工具箱，核心目标是：

- 在多视图图像中重建 3D 线地图（line mapping）
- 基于点+线联合约束做定位（hybrid localization）
- 为 2D/3D 线相关几何操作提供可复用 API

实现形态是典型的 **Python 编排 + C++ 计算内核（pybind11）** 混合架构。

### 1.2 工程分层结构

#### 1.2.1 入口/应用层

- `runners/`：按数据集与任务组织的可执行脚本（Hypersim、ScanNet、ETH3D、COLMAP、7Scenes、Cambridge、InLoc 等）
- `scripts/`：下载数据、评测、格式转换等辅助脚本
- `visualize_3d_lines.py`：重建结果可视化入口

#### 1.2.2 流程编排层（Python）

- `src/limap/runners/`：主流程接口
- 代表入口：
  - `line_triangulation`（RGB-only 三角化重建）
  - `line_fitnmerge`（有深度时的 fit & merge）
  - `hybrid_localization`（点线联合定位）
- `src/limap/util/config.py`：YAML 读取、配置继承、命令行动态覆写
- `src/limap/util/io.py`：中间结果与最终结果读写

#### 1.2.3 算法模块层（Python API + C++绑定）

- `src/limap/base`：几何基础结构（Camera / CameraPose / CameraImage / ImageCollection / Line2d / Line3d / LineTrack）
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

#### 1.2.4 原生内核层（C++）

- `src/limap/internal/bindings.cc` 统一创建 `_limap` Python 扩展模块
- 各子模块 C++ 内核通过 pybind 暴露为 `_limap._base/_triangulation/_merging/...`
- `CMakeLists.txt` + `scikit-build-core`（`pyproject.toml`）负责构建

#### 1.2.5 配置与文档层

- `cfgs/`：任务/数据集配置（triangulation、fitnmerge、localization、eval 等）
- `docs/`：教程与 API 文档
- `tests/`：基础与第三方依赖相关测试

### 1.3 按数据流看模块职责

#### 1.3.1 输入与相机组织

- 数据集 loader（如 `runners/hypersim/loader.py`）把原始文件转换为 `ImageCollection`
- `ImageCollection` 是后续流程统一输入载体（相机内外参 + 图像路径 + 图像 id）

#### 1.3.2 SfM 辅助元信息

- `limap.runners.compute_sfminfos` 通过 `limap.pointsfm` 调 COLMAP/hloc：
  - 计算每张图的视觉邻居（neighbors）
  - 估计场景 3D robust ranges

#### 1.3.3 2D 线前端

- `limap.runners.compute_2d_segs`：检测 2D 线段（可选描述子）
- `limap.runners.compute_matches` / `compute_exhaustive_matches`：线匹配
- `line2d/register_*`：按配置选择 detector/extractor/matcher

#### 1.3.4 多视图线三角化

- `GlobalLineTriangulator`：候选生成、打分、连边、聚类、轨迹聚合
- 候选来源：
  - 纯代数线三角化
  - one-point / many-points（可借助 point SfM 的点线二部图）
  - VP 方向先验

#### 1.3.5 后处理与优化

- `merging`：重投影过滤、remerge、敏感性过滤、重叠过滤
- `optimize`：线 BA / 混合 BA（按配置开关）

#### 1.3.6 结果落盘与可视化

- `save_folder_linetracks_with_info`：保存 track 文件 + 配置 + imagecols + 2D 线
- `save_obj`：导出 OBJ 线模型
- `visualize_3d_lines.py`：离线可视化

---

## 2. Quickstart 主线：从命令到结果目录

### 2.1 目标命令

```bash
python runners/hypersim/triangulation.py --output_dir outputs/quickstart_triangulation
```

以下内容按真实调用链展开。

### 2.2 入口脚本阶段（`runners/hypersim/triangulation.py`）

#### 2.2.1 `parse_config()`

- 默认配置链：
  - 主配置：`cfgs/triangulation/hypersim.yaml`
  - 默认底座：`cfgs/triangulation/default.yaml`
- `parse_known_args()` 仅显式解析少量参数（`config_file/default_config_file/npyfolder`）
- 其他参数（例如 `--output_dir`）进入 `unknown`，由 `limap.util.config.update_config` 按 key-path 回写配置
- 快捷参数映射：`-nv -> --n_visible_views`、`-nn -> --n_neighbors`、`-sid -> --scene_id`

#### 2.2.2 `main()`

- 构建数据集对象：`dataset = Hypersim(cfg["data_dir"])`
- 调用：`run_scene_hypersim(cfg, dataset, cfg["scene_id"], cam_id=cfg["cam_id"])`

### 2.3 Hypersim 数据读取（`runners/hypersim/loader.py` + `runners/hypersim/Hypersim.py`）

`read_scene_hypersim(..., load_depth=False)` 关键动作：

1. `dataset.set_scene_id(scene_id)`：定位场景目录并读取 `meters_per_asset_unit`
2. `dataset.set_max_dim(cfg["max_image_dim"])`：按最大边缩放分辨率与内参
3. 构建候选帧 id：`range(0, input_n_views, input_stride)`，再 `filter_index_list` 过滤实际存在图像
4. 读取相机轨迹（`camera_keyframe_positions/orientations.hdf5`），并做坐标系变换（含 `R180x`）
5. 构建：
  - `base.Camera("SIMPLE_PINHOLE", K, ...)`
  - 每帧一个 `base.CameraImage`
  - 最终 `base.ImageCollection`
6. 因为 `load_depth=False`，该路径不加载深度

输出：`imagecols`

### 2.4 主流程 `limap.runners.line_triangulation`（A-F）

文件：`src/limap/runners/line_triangulation.py`

#### A. setup + SfM 元信息

1. `runners.setup(cfg)`：
  - 设置 `dir_save`（本例为 `outputs/quickstart_triangulation`）
  - 设置 `dir_load`
2. 去畸变检查：
  - 若 `imagecols` 非 undistorted，则跑 `undistort_images`
  - Hypersim 常用 `SIMPLE_PINHOLE`，一般可视为已去畸变
3. 先保存基础信息：
  - `image_list.txt`
  - `imagecols.npy`
4. `compute_sfminfos(cfg, imagecols)`：
  - 因为 `neighbors` 未显式传入，会触发 COLMAP 已知位姿三角化
  - 调用链：
    - `pointsfm.run_colmap_sfm_with_known_poses`
    - 内部 `run_hloc_matches` 做点特征提取与匹配
    - `pycolmap.triangulate_points` 做点云三角化
  - 基于点云模型计算：
    - `neighbors`（每图前 `n_neighbors` 个视觉邻居）
    - `ranges`（场景稳健空间范围）
  - 写出 `metainfos.txt`

#### B. 2D 线段检测

1. `compute_descinfo` 判定：
  - quickstart 组合通常是：
    - `triangulation.use_exhaustive_matcher = True`
    - `line2d.compute_descinfo = False`
  - 结果：**只做检测，不做描述子提取**
2. `compute_2d_segs`：
  - detector 使用 `lsd`（来自 `hypersim.yaml`）
  - 输出目录：`line_detections/lsd/segments`
  - 若 `line2d.visualize=True`（默认 true），还会写检测可视化图

#### C. 线匹配阶段

- 仅当 `use_exhaustive_matcher=False` 才会跑 `compute_matches`
- quickstart 为 `True`，所以这里跳过
- 后续三角化对每对邻居图采用“线段全配对穷举”

#### D. 多视图三角化核心

1. 初始化：
  - `Triangulator = triangulation.GlobalLineTriangulator(cfg["triangulation"])`
  - `Triangulator.SetRanges(ranges)`
  - `Triangulator.Init(all_2d_lines, imagecols)`
2. 可选分支（quickstart 默认关闭）：
  - `use_vp=False`：不注入 VP
  - `use_pointsfm.enable=False`：不注入点线二部图/3D 点
3. 主循环：
  - 对每个 `img_id` 调 `TriangulateImageExhaustiveMatch(img_id, neighbors[img_id])`
4. C++ 内核关键逻辑（`base_line_triangulator.cc` + `global_line_triangulator.cc`）：
  - 对图像 i 的每条 2D 线，与每个邻居图 j 的每条 2D 线组成候选
  - 候选先过几何筛选（线长、退化角、epipolar IoU、敏感度）
  - 通过后做三角化（默认 algebraic）并估计不确定度
  - `ScoringCallback` 按跨视图支持打分
  - 保留高分连接，构图聚类（默认 `merging_strategy=greedy`）
  - 聚类后聚合成 `LineTrack`
5. 过滤：
  - `filter_tracks_by_reprojection`
  - 可选 `remerge`（quickstart 默认启用）
  - `filter_tracks_by_sensitivity`
  - `filter_tracks_by_overlap`
  - 可见视角阈值筛选（`n_visible_views`）

#### E. 几何优化（BA）

- quickstart 覆写 `refinement.disable=True`，因此 **本次不跑 BA**

#### F. 结果保存与可视化

1. 保存：
  - `alltracks.txt`（可见帧数 >= 4 的 track 摘要）
  - `finaltracks/`（逐 track 文本 + 附加信息）
  - `triangulated_lines_nv{n_visible_views}.obj`
2. 若 `visualize=True`（默认 true）：
  - 先 `input("Press Enter...")` 等待交互
  - 再弹出 Open3D 可视化窗口

### 2.5 典型输出目录结构

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
  undistorted_images/      # 仅输入需去畸变时出现
```

说明：

- 因为 `use_exhaustive_matcher=True`，通常不会生成 `line_matchings/...`
- 因为 `refinement.disable=True`，不会有 BA 后的新 track 分支

### 2.6 一句话时序图

`triangulation.py`（解析配置）  
-> `Hypersim loader`（构建 ImageCollection）  
-> `line_triangulation`  
-> `setup`  
-> `compute_sfminfos(COLMAP+hloc)`  
-> `compute_2d_segs(LSD)`  
-> `GlobalLineTriangulator`（穷举邻居线对 + 三角化 + 打分聚类）  
-> `filters/remerge`  
-> `save finaltracks + obj`  
-> `Open3D visualize(可选)`

### 2.7 quickstart 的工程取舍

- 优先可跑性：`use_cuda=False`、`lsd`、`compute_descinfo=False`
- 用穷举线对替代描述子匹配：`use_exhaustive_matcher=True`
- 默认关闭 BA：`refinement.disable=True`

因此它是一个“最小闭环演示路径”：

- 保留完整主流程
- 降低外部模型依赖与配置复杂度
- 在 100 张示例图上快速出结果

---

## 3. Line2D 前端组件（detector / extractor / matcher）

### 3.1 总体结构与 quickstart 影响

LIMAP 的 line2d 前端采用“注册器 + 多后端实现”插件化结构，入口为：

- detector 注册：`src/limap/line2d/register_detector.py`
- extractor 注册：`src/limap/line2d/register_detector.py`
- matcher 注册：`src/limap/line2d/register_matcher.py`

关键点：若 `triangulation.use_exhaustive_matcher=True`（quickstart 默认），三角化会跳过 line matcher，直接做邻居图线段穷举；此时 extractor/matcher 影响下降，detector 更关键。

### 3.2 Detector 实现（5 种）

1. `lsd`（`src/limap/line2d/LSD/lsd.py`）
  - 原理：经典 LSD（梯度 + 统计显著性），非深度学习
  - 实现：`pytlsd.lsd(img)`
  - 工程效果：CPU 友好、速度快、依赖轻；弱纹理/复杂光照下通常不如学习法稳
2. `sold2`（`src/limap/line2d/SOLD2/sold2.py` + `sold2_wrapper.py`）
  - 原理：端到端线段检测+描述；匹配侧使用 Needleman-Wunsch 思路
  - 实现：输出线段、descriptor、heatmap，线得分来自沿线 saliency 聚合
  - 工程效果：复杂场景更稳，但模型重、推理慢，依赖 CUDA/权重
3. `deeplsd`（`src/limap/line2d/DeepLSD/deeplsd.py`）
  - 原理：学习式线检测（结合深度预测与 LSD 风格几何约束）
  - 实现：`DeepLSD(conf)`，封装中常用线长近似 score
  - 工程效果：鲁棒性较强；比 LSD 重；可回落 CPU（速度会明显下降）
4. `hawpv3`（`src/limap/line2d/HAWPv3/hawp.py`）
  - 原理：wireframe 思路，联合建模 junction 与 line
  - 实现：读取 HAWPv3 权重后按阈值过滤
  - 工程效果：结构化场景（室内/建筑）常有优势；依赖额外安装（`misc/install/hawpv3.md`）
5. `tp_lsd`（`src/limap/line2d/TP_LSD/tp_lsd.py`）
  - 原理：学习式线检测 + 亮度/纹理增强预处理
  - 实现：先做 HSV-V 增强与平滑，再送网络；score 常用线长近似
  - 工程效果：复杂光照纹理下较稳；安装门槛较高（`misc/install/tp_lsd.md`）

### 3.3 Extractor 实现（7 种）

1. `sold2`
  - 文件：`src/limap/line2d/SOLD2/sold2.py`
  - 原理：复用 SOLD2 descriptor 特征图，对每条线采样并聚合
  - 效果：与 SOLD2 matcher 配套最好；速度/显存开销中高
2. `lbd`
  - 文件：`src/limap/line2d/LBD/extractor.py`
  - 原理：传统 LBD，多尺度金字塔 + 线带描述
  - 效果：CPU 友好、工程稳定；区分性常弱于学习描述子
3. `l2d2`
  - 文件：`src/limap/line2d/L2D2/extractor.py`
  - 原理：线附近裁 48x32 归一化 patch，经 CNN 输出 128D
  - 效果：判别力优于传统描述子；有额外裁剪+前向耗时
4. `linetr`
  - 文件：`src/limap/line2d/LineTR/extractor.py` + `line_transformer.py`
  - 原理：SuperPoint 密集特征 + LineTR Transformer 聚合子线 token
  - 效果：判别力强；显存/时延更高
5. `superpoint_endpoints`
  - 文件：`src/limap/line2d/endpoints/extractor.py`
  - 原理：仅提取线段端点 SuperPoint 描述，再在 matcher 侧线级组合
  - 效果：实现简单、速度快；端点定位误差敏感
6. `wireframe`
  - 文件：`src/limap/line2d/GlueStick/extractor.py`
  - 原理：SuperPoint + `lines_to_wireframe`，构建 junction/line 结构化输入
  - 效果：为 GlueStick 提供上下文，鲁棒性较好；流程更重
7. `dense_naive`
  - 文件：`src/limap/line2d/dense/extractor.py`
  - 原理：不显式提线描述子，只保存图像/线段/分数给 dense matcher
  - 效果：提取阶段开销低，但依赖后续 dense_roma 计算

### 3.4 Matcher 实现（8 种）

1. `sold2`
  - 文件：`src/limap/line2d/SOLD2/sold2.py` + `SOLD2/model/line_matching.py`
  - 原理：线段采样 + Needleman-Wunsch/序列对齐 + 可选互检
  - 效果：对遮挡/局部错位更有韧性；计算中高
2. `lbd`
  - 文件：`src/limap/line2d/LBD/matcher.py`
  - 原理：`pytlbd.lbd_matching_multiscale` 多尺度匹配
  - 效果：CPU 高效稳定；能力偏传统
3. `l2d2`
  - 文件：`src/limap/line2d/L2D2/matcher.py`
  - 原理：相似度矩阵 + mutual nearest neighbor（或 top-k）
  - 效果：实现简洁、速度可控；受 extractor 质量影响大
4. `linetr`
  - 文件：`src/limap/line2d/LineTR/matcher.py`
  - 原理：subline 距离映射到 keyline 距离后阈值+互检
  - 效果：中高难视角下精度较好；代价偏高
5. `nn_endpoints`
  - 文件：`src/limap/line2d/endpoints/matcher.py`
  - 原理：端点相似度 -> 线级组合 -> Sinkhorn OT -> 线匹配
  - 效果：速度快、配置简单，常作为高效基线
6. `superglue_endpoints`
  - 文件：`src/limap/line2d/endpoints/matcher.py`
  - 原理：先 SuperGlue 端点匹配，再聚合到线级 + Sinkhorn
  - 效果：鲁棒性强于纯 NN 端点；耗时更高
7. `gluestick`
  - 文件：`src/limap/line2d/GlueStick/matcher.py`
  - 原理：junction + line + connectivity 图匹配网络
  - 效果：结构化场景表现通常更稳；依赖和算力开销高
8. `dense_roma`
  - 文件：`src/limap/line2d/dense/matcher.py` + `dense_matcher/roma.py`
  - 原理：先用 RoMa 估计 dense warp/certainty，再沿线采样按几何距离与重叠率筛选对应
  - 效果：对大视角变化/重复纹理有优势；计算重，需安装 `romatch`（`misc/install/roma.md`）

### 3.5 兼容配对关系（必须对齐）

- `sold2 extractor` ↔ `sold2 matcher`
- `lbd extractor` ↔ `lbd matcher`
- `l2d2 extractor` ↔ `l2d2 matcher`
- `linetr extractor` ↔ `linetr matcher`
- `superpoint_endpoints extractor` ↔ `nn_endpoints` / `superglue_endpoints`
- `wireframe extractor` ↔ `gluestick matcher`
- `dense_naive extractor` ↔ `dense_roma matcher`

### 3.6 选型建议（结合现有配置）

1. 极致快速/快速验证
  - 组合：`lsd + superpoint_endpoints + nn_endpoints`
  - 配置：`cfgs/triangulation/default_fast.yaml`
  - 特点：速度优先，性能会下降
2. 纯 CPU 场景
  - 组合：`lsd + lbd + lbd`
  - 配置：`cfgs/triangulation/default_cpu.yaml`
  - 特点：无重型深度模型依赖，部署门槛低
3. 默认鲁棒配置
  - 组合：`deeplsd + wireframe + gluestick`
  - 配置：`cfgs/triangulation/default.yaml`
  - 特点：精度/鲁棒性优先，资源需求高
4. quickstart 当前命令
  - 本质：`lsd` 检测 + 三角化阶段穷举线对（`use_exhaustive_matcher=True`）
  - 影响：extractor/matcher 非主瓶颈，detector 与三角化参数更关键

### 3.7 “效果结论”的边界

仓库内没有统一同协议的 detector/extractor/matcher 全量 benchmark。  
本节“效果”结论是基于实现机制与默认配置的工程判断，适合第一轮选型。若要严谨对比，建议在同一数据集、同一阈值协议下做 ablation（可从 `cfgs/triangulation/default*.yaml` 派生）。

---

## 4. Triangulation 方案拆解（Base 提案生成 + Global 成轨）

这一章只聚焦 `src/limap/triangulation/` 及其直接依赖，按“代码真实执行顺序”整理工程里的三角化方案。

### 4.1 模块边界与分工

1. Python 编排层（流程入口）
  - 位置：`src/limap/runners/line_triangulation.py`
  - 职责：
    - 构造 `GlobalLineTriangulator(cfg["triangulation"])`
    - 注入场景范围 `SetRanges(ranges)`
    - 注入可选先验：`InitVPResults`、`SetBipartites2d`、`SetSfMPoints`
    - 逐图调用 `TriangulateImage` / `TriangulateImageExhaustiveMatch`
    - 最后 `ComputeLineTracks()`
2. C++ 三角化内核
  - 基类：`src/limap/triangulation/base_line_triangulator.{h,cc}`
  - 派生类：`src/limap/triangulation/global_line_triangulator.{h,cc}`
  - 职责：
    - 基类负责“候选 3D 线提案生成”
    - 派生类负责“跨视图打分、筛边、聚类成 track”
3. 几何原语与求解函数
  - 位置：`src/limap/triangulation/functions.{h,cc}`
  - 职责：
    - 基础几何（法向、VP 方向、E/F、epipolar IoU）
    - 点/线三角化具体算子（代数、端点、已知点、已知方向）
4. 轨迹合并与线段聚合
  - 位置：
    - `src/limap/merging/merging.cc`
    - `src/limap/merging/aggregator.cc`
  - 职责：
    - 图聚类 label 生成（greedy/exhaustive/avg）
    - 每个 track 的多条 3D 线聚合成最终一条

### 4.2 核心数据结构（决定了流程形态）

位置：`src/limap/triangulation/base_line_triangulator.h`

1. 节点与候选定义
  - `LineNode = (img_id, line_id)`：图节点
  - `TriTuple = (Line3d, score, (ng_img_id, ng_line_id))`
    - `score` 在提案阶段先占位 `-1`，由 Global 阶段打分回填
2. 运行期容器
  - `edges_[img][line]`：当前节点的候选连接列表（邻图线段）
  - `tris_[img][line]`：当前节点所有三角化提案（多个来源）
  - `neighbors_[img]`：当前图参与的邻接图像 id 列表
  - `tracks_`：最终 `LineTrack` 输出
3. Global 扩展容器
  - `valid_edges_`：通过打分阈值后的连接
  - `tris_best_`：每个节点分数最高的提案（后续建图主用）
  - `valid_flags_`：迭代剪枝后保留节点标记

### 4.3 入口时序（从匹配到三角化）

位置：`src/limap/runners/line_triangulation.py`

1. 初始化
  - `Triangulator = triangulation.GlobalLineTriangulator(cfg["triangulation"])`
  - `Triangulator.SetRanges(ranges)`
  - `Triangulator.Init(all_2d_lines, imagecols)`
2. 可选先验
  - `use_vp=True`：先跑 VP 检测，再 `InitVPResults(vpresults)`
  - `use_pointsfm.enable=True`：构造点线二部图并 `SetBipartites2d`，可选 `SetSfMPoints`
3. 逐图三角化入口二选一
  - `use_exhaustive_matcher=True`：
    - `TriangulateImageExhaustiveMatch(img_id, neighbors[img_id])`
    - 邻图内线段对穷举
  - `use_exhaustive_matcher=False`：
    - 读取 `matches_{img_id}.npy`
    - `TriangulateImage(img_id, matches)`
4. 全局聚类成轨
  - `linetracks = Triangulator.ComputeLineTracks()`

### 4.4 Base 阶段：候选 3D 线提案怎么来

位置：`src/limap/triangulation/base_line_triangulator.cc`

`triangulateOneNode(img_id, line_id)` 是核心。对每个 `(l1, l2)` 邻接线对，会并行尝试多种 proposal；所有通过闸门的结果都写入 `tris_[img_id][line_id]`。

1. 前置过滤
  - 2D 长度门槛：`min_length_2d`（`l1` 或 `l2` 太短直接跳过）
2. Proposal A：PointSfM 约束（可选）
  - 触发条件：`use_pointsfm_ == true`
  - A1 many-points（三角化多个共享 3D 点后 TLS 拟合）
    - 开关：`disable_many_points_triangulation`
    - 条件：共享点数量 `>= 2`
  - A2 one-point（已知线上一点）
    - 开关：`disable_one_point_triangulation`
    - 条件：至少有一个共享点
3. Proposal B：VP 方向先验（可选）
  - 触发条件：`use_vp=true` 且该线有 VP
  - 开关：`disable_vp_triangulation`
  - 算子：`triangulate_line_with_direction`
4. Proposal C：纯两视图代数线三角化（默认主路径）
  - 开关：`disable_algebraic_triangulation`
  - 在求解前有三道几何闸门：
    - 退化角过滤：`line_tri_angle_threshold`
    - 弱极线一致性：`IoU_threshold`（`compute_epipolar_IoU`）
    - 不稳定线过滤：`sensitivity_threshold`
  - 求解器二选一：
    - 默认：`triangulate_line`（射线-平面）
    - 若 `use_endpoints_triangulation=true`：`triangulate_line_by_endpoints`
5. 后置统一处理
  - 计算不确定度：`line.computeUncertainty(view, var2d)`，取两视图最小值
  - 若配置了 `SetRanges(ranges)`，执行空间范围门槛 `test_line_inside_ranges`
  - 通过者写入 `TriTuple(line, -1.0, (ng_img_id, ng_line_id))`

### 4.5 Global 阶段：如何给 proposal 打分并筛连接

位置：`src/limap/triangulation/global_line_triangulator.cc`

1. 调度入口
  - `ScoringCallback(img_id)` 遍历该图所有 `line_id`
  - 每个节点调用 `scoreOneNode`
2. 节点打分（`scoreOneNode`）
  - 对当前节点的每个 proposal `tri_i`：
    - 与同节点其余 proposal `tri_j` 两两比较
    - 仅统计来自“不同邻图”的支持
    - 每一邻图最多贡献一个支持分（取该邻图内最大分）
  - 单对比较分数：
    - `score3d = linker.compute_score_3d(l_i, l_j)`
    - `score2d = linker.compute_score_2d(proj(l_i->ng_view), ng_line2d)`
    - `score = min(score3d, score2d)`
  - 节点最终分：对各邻图 best support 求和
3. 有效候选筛选
  - 先按分数降序
  - 只保留前 `max_valid_conns`
  - 仅分数 `>= fullscore_th` 的 proposal 才进入 `valid_tris_` 与 `valid_edges_`
4. 代表提案选择
  - 每节点取分数最大 proposal 存到 `tris_best_`
  - 后续图聚类与 track 构建基本依赖这条“代表线”

### 4.6 图构建与迭代剪枝

位置：`src/limap/triangulation/global_line_triangulator.cc`

1. 节点先做“最小外边数”过滤
  - 参数：`min_num_outer_edges`
  - 实现：`filterNodeByNumOuterEdges`
  - 机制：
    - 先统计每节点当前有效外连数
    - 低于阈值节点标记为无效
    - 通过 parent 反向邻接做队列传播，迭代扣减并继续淘汰
2. 构建无向边集合
  - 从 `valid_edges_` 收集 `(node1, node2)`，去重后建图
  - 两端节点必须都在 `valid_flags_` 里有效
3. 边分数
  - 计算了 `score_3d` 与双向投影 `score_2d`
  - 当前实现最终 `score = score_3d`（`score_2d` 计算后未用于最终边权）
  - 只有 `score > 0` 才入图

### 4.7 聚类成 Track 与 3D 线聚合

位置：

- `src/limap/triangulation/global_line_triangulator.cc` -> `build_tracks_from_clusters`
- `src/limap/merging/merging.cc`
- `src/limap/merging/aggregator.cc`

1. 轨迹标签策略（`merging_strategy`）
  - `greedy`：最大边优先的并查集合并（快，约束较松）
  - `exhaustive`：合并前对两簇内线段做更严格两两一致性检查
  - `avg`：用簇平均线做兼容性检查，折中速度与约束
2. 回填 `LineTrack` 字段
  - 对每个图节点：
    - 写入 `image_id_list` / `line_id_list` / `line2d_list` / `line3d_list` / `score_list`
3. 轨迹最终 3D 线
  - `Aggregator::aggregate_line3d_list(...)`
  - `num_outliers_aggregator` 用于端点投影的两端截断，降低离群影响

### 4.8 配置项按“作用层级”归组

参考：`cfgs/triangulation/default.yaml`

1. 提案生成层（Base）
  - `min_length_2d`
  - `use_endpoints_triangulation`
  - `line_tri_angle_threshold`
  - `IoU_threshold`
  - `sensitivity_threshold`
  - `var2d`
  - `use_vp` + `vpdet_config`
  - `use_pointsfm.*`
  - `disable_*_triangulation` 四个开关
2. 提案筛分层（Global）
  - `fullscore_th`
  - `max_valid_conns`
  - `min_num_outer_edges`
  - `linker2d_config`
  - `linker3d_config`
3. 轨迹构建层（Global + merging）
  - `merging_strategy`
  - `num_outliers_aggregator`
4. 后处理层（runner/merging）
  - `filtering2d.*`
  - `remerging.*`
  - `n_visible_views`

### 4.9 调参顺序（针对“三角化本体”）

1. 先稳提案质量（减少伪 3D 线）
  - 提高 `line_tri_angle_threshold`
  - 提高 `IoU_threshold`
  - 降低 `sensitivity_threshold`（更严格）
2. 再收全局筛分（减少误连接）
  - 提高 `fullscore_th`
  - 适当降低 `max_valid_conns`
  - 提高 `min_num_outer_edges`
3. 最后改聚类策略与聚合鲁棒性
  - `greedy` 不稳时换 `avg` / `exhaustive`
  - 适当增大 `num_outliers_aggregator`

---

## 6. 误检与误匹配抑制：多级闸门 + 调参顺序

你提到的“纹理密集导致重复检测、误匹配”，在 LIMAP 中不是单模块处理，而是多级闸门串联。

### 6.1 检测阶段：先压重复与低质量线

1. 检测后可选线段合并（去重复）
  - 开关：`line2d.do_merge_lines`
  - 位置：`src/limap/line2d/base_detector.py` -> `merge_lines`
  - 调用链：
    - `BaseDetector.detect_all_images` 在 `detect` 后、`take_longest_k` 前调用 `self.merge_lines(segs)`
    - `self.merge_lines` 先把检测结果从 `(N,5)` 裁成端点 `(N,2,2)`（只保留 `x1,y1,x2,y2`，不保留 score），然后调用 `line_utils.merge_lines`
    - 当前入口未传阈值参数，因此使用默认 `merge_lines(lines, thresh=5.0, overlap_thresh=0.0)`
  - `merge_lines.py` 实现细节（按执行顺序）：
    - 第 1 步：两两计算“线到线正交距离 + 重叠率”
      - 调用 `get_orth_line_dist(lines, lines, return_overlap=True)`
      - `project_point_to_line`：把一组线段端点正交投影到另一组线的方向上，得到
        - 投影 1D 坐标 `coords1d`
        - 到目标线的正交距离 `dist_to_line`
      - 对任意线段对 `(Li, Lj)`，距离项取双向平均：
        - `Lj` 的两个端点到 `Li` 的距离和
        - `Li` 的两个端点到 `Lj` 的距离和
        - 两者再取平均，得到对称的 `orth_dist[i,j]`
      - `get_segment_overlap`：把投影到参考线上的端点坐标排序后，计算与区间 `[0,1]` 的交长度（即“在线段范围内的重叠比例”）
      - 重叠率用双向平均 `overlaps=(overlaps1+overlaps2)/2`；同时保留双向最小值 `min_overlaps`（在非 `return_overlap` 场景会用于严格过滤）
    - 第 2 步：构图（邻接矩阵）
      - 默认 `overlap_thresh=0.0`：
        - 只有 `overlaps > 0` 且 `orth_dist < thresh` 才连边
      - 若设置 `overlap_thresh > 0`：
        - 额外计算两条线四组端点距离的最小值 `close_endpoint`
        - 连边条件变为 `((overlaps > 0) | (close_endpoint < overlap_thresh)) & (orth_dist < thresh)`
        - 即允许“端点很近但本体不重叠”的共线段被合并
    - 第 3 步：连通分量聚类
      - 用 `scipy.sparse.csgraph.connected_components(..., directed=False)` 对无向图做连通分量分解
      - 每个连通分量是一簇待合并线段（具备传递性：A 连 B、B 连 C 会并到同簇）
    - 第 4 步：簇内几何合并 `merge_line_cluster`
      - 输入：簇内线段 `(n,2,2)`，将全部端点展平为 `2n` 个点
      - 主方向估计：
        - 每条线按长度加权（长线权重大，且同一线的两个端点重复该权重）
        - 计算加权协方差矩阵，并解析求 2x2 对称矩阵主特征向量 `u`
      - 中心线构造：
        - 取所有端点几何中心 `cross = mean(points)` 作为中心点
        - 定义方向线 `cross + t*u`
      - 端点重建：
        - 将所有端点投影到该方向线上，取最小/最大投影位置对应的两点作为新线段端点
      - 输出：该簇合并后的一条线段 `(2,2)`
    - 第 5 步：汇总输出
      - 所有簇分别合并后 `np.stack` 为新线段集合 `(M,2,2)`
      - 回到 `BaseDetector.merge_lines` 再 reshape 为 `(M,4)`，供后续 `take_longest_k` 使用
  - 实际效果与边界：
    - 主要去掉“同方向、同位置附近”的重复检测和碎片化线段
    - 默认 `overlap_thresh=0` 时，不会合并“仅平行接近但不重叠”的线段
    - 合并发生在 detector 后处理阶段，且在截断 top-K 之前，因此会先减少重复，再做长度筛选
2. 长度优先截断（抑制短碎线）
  - 参数：`line2d.max_num_2d_segs`
  - 位置：`BaseDetector.take_longest_k`
  - 作用：按长度保留 top-K，短纹理碎线会被大量清理
3. detector 内部阈值
  - 例如 HAWPv3 的 `thresh`，以及 SOLD2/TP-LSD/DeepLSD 的检测阈值
  - 作用：先剔除低置信线段

### 6.2 匹配前约束：减少不该比较的组合

1. 邻居图裁剪（不是全图互配）
  - 参数：`n_neighbors`、`sfm.min_triangulation_angle`、`sfm.neighbor_type`
  - 位置：`src/limap/pointsfm/functions.py`
  - 作用：只在 SfM 共视邻居间匹配，减少远距误配
2. 点特征几何验证（用于构建邻居）
  - 位置：`src/limap/pointsfm/colmap_sfm.py` 的 `triangulation.estimation_and_geometric_verification`
  - 作用：先剔除几何不一致点匹配，使邻居关系更干净

### 6.3 线匹配阶段：匹配器内部抑制误配

1. 通用入口与控制面（`BaseMatcher` + 注册器）
  - 位置：
    - `src/limap/line2d/register_matcher.py`
    - `src/limap/line2d/base_matcher.py`
  - 实现：
    - `register_matcher.get_matcher` 按 `line2d.matcher.method` 分发到 `sold2/lbd/l2d2/linetr/nn_endpoints/superglue_endpoints/gluestick/dense_roma`
    - 统一参数 `line2d.matcher.topk` 注入到 matcher；多数实现采用：
      - `topk == 0`：走“更严格”的一对一分支（Mutual NN / Cross-check / OT 后唯一匹配）
      - `topk > 0`：走“候选扩展”分支（每条线保留 top-k 候选，通常不再做互检）
    - 输出统一为 `(N,2)` 的线对索引；空输入通常返回空数组

2. Mutual NN（Mutual nearest neighbor） / Cross-check 具体落地
  - `L2D2`（`src/limap/line2d/L2D2/matcher.py`）
    - 相似度矩阵：`score_mat = desc1 @ desc2.T`
    - 严格分支（`topk==0`）：
      - `nearest1 = argmax(score_mat, axis=1)`，`nearest2 = argmax(score_mat, axis=0)`
      - 仅保留 `nearest2[nearest1] == arange(len(desc1))` 的双向最近邻
    - 候选分支（`topk>0`）：
      - 每行按分数取前 `topk`，不做互检/阈值
  - `LineTR`（`src/limap/line2d/LineTR/matcher.py` + `LineTR/nn_matcher.py`）
    - 关键线 (keyline)：原始检测到的整条线段（输入线）
    - 子线 (subline)：把一条关键线按 token 上限切成的若干段，便于 Transformer 编码长线
    - 先算子线距离：`get_dist_matrix` 用 `2 - 2 * dot`（裁剪到非负）
    - 再聚合到关键线：`subline2keyline(mat0 @ dist @ mat1.T)`
    - 严格分支（`topk==0`）：
      - `nn_matcher_distmat(..., nn_threshold, is_mutual_NN=True)`
      - 同时要求“距离 < nn_threshold + 双向最近邻”
    - 候选分支（`topk>0`）：
      - 每行取最小距离 top-k（最近邻），不再做 mutual/阈值门槛
  - `SOLD2`（`src/limap/line2d/SOLD2/sold2_wrapper.py` + `SOLD2/model/line_matching.py`）
    - 核心是 `WunschLineMatcher.compute_matches`
    - 先把每条线采样成点序列（`num_samples`、`min_dist_pts`），点描述子两两打分
    - 线对预筛：先按“点级最大响应的双向平均”取 `top_k_candidates`
    - 精筛：对候选执行 Needleman-Wunsch 动态规划（含反向线段），选每条线最佳匹配
    - `cross_check=True`（默认配置）时做反向互检，不互为最佳则置 `-1`

3. Sinkhorn / OT （Optimal Transport）约束（endpoints 系）
  - 公共 OT 位置：`src/limap/point2d/superglue/superglue.py`
    - `log_optimal_transport` + `_get_matches`
    - `_get_matches` 内部同时做 mutual 检查与 `match_threshold`（默认 0.2）过滤
  - `nn_endpoints`（`src/limap/line2d/endpoints/matcher.py`）
    - 先用端点描述子点积得到端点两两分数
    - 再合成为线对分数：
      - `0.5 * max(d00+d11, d01+d10)`（端点顺序不敏感）
    - 严格分支（`topk==0`）：对线对分数跑 OT，再由 `_get_matches` 输出唯一匹配
    - 候选分支（`topk>0`）：直接按线对分数取 top-k，不跑 OT/阈值
  - `superglue_endpoints`（同文件）
    - 先跑 `SuperGlue(inputs)` 得到上下文增强后的点分数 `out["scores"]`
    - 之后与 `nn_endpoints` 相同：线对分数重排 -> OT -> `_get_matches`
    - `topk>0` 同样直接取 top-k，绕过 OT 过滤

4. 图结构匹配（GlueStick）
  - 位置：`src/limap/line2d/GlueStick/matcher.py`
  - 输入不仅有线段，还包含 junction、junction descriptor、`lines_junc_idx` 图结构
  - 严格分支（`topk==0`）：
    - 调 `GlueStick` 模型，直接读取 `out["line_matches0"]`
    - 仅保留 `!= -1` 的一对一匹配
  - 候选分支（`topk>0`）：
    - 读取 `raw_line_scores`，按行取 top-k
    - wrapper 层不再加 mutual/阈值

5. dense matcher 的双向一致性与重叠门槛（`dense_roma`）
  - 位置：`src/limap/line2d/dense/matcher.py`
  - 核心流程：
    - 通过 RoMa 得到对称 warping：`1->2` 与 `2->1`，以及 certainty 图
    - 每条线均匀采样 `n_samples` 点，warp 到目标图后计算：
      - 到候选线的垂距
      - 投影是否落在线段范围内（overlap）
      - certainty 是否高于 `sample_thresh`
    - 仅对“高 certainty 且在重叠区”的样本加权求平均距离
    - 若线段有效重叠比例 `< segment_percentage_th`，该线对距离置大（拒绝）
    - 双向融合后再加硬门槛：
      - `min(overlap_1to2, overlap_2to1) >= segment_percentage_th`
      - `max(dist_1to2, dist_2to1) <= pixel_th`
  - 匹配输出：
    - 默认 `one_to_many=False` 时，保留每条线的行最小距离（一对一倾向）
    - `topk>0` 分支当前实现与默认分支等价，仍是“阈值内全部保留”，未显式截断 top-k

6. LBD 的实现边界
  - 位置：`src/limap/line2d/LBD/matcher.py`
  - 严格分支（`topk==0`）：
    - 直接调用 `pytlbd.lbd_matching_multiscale(...)`
    - RuntimeError 时回退为空匹配
  - 候选分支（`topk>0`）：
    - 当前未实现（`NotImplementedError`）

7. 参数如何影响“抑制误配”强度（对应源码行为）
  - `line2d.matcher.topk = 0`：
    - 倾向唯一匹配，更多使用 mutual/cross-check/OT 过滤，误配率通常更低
  - `line2d.matcher.topk > 0`：
    - 候选覆盖更高，但多数 matcher 会放宽约束并输出一对多候选
  - method-specific 关键阈值：
    - `LineTR`: `nn_threshold`
    - endpoints OT: `SuperGlue.match_threshold`
    - `dense_roma`: `segment_percentage_th`、`pixel_th`、`sample_thresh`
    - `SOLD2`: `cross_check`、`top_k_candidates`、`num_samples/min_dist_pts`

### 6.4 三角化阶段：几何闸门过滤假匹配

位置：`src/limap/triangulation/base_line_triangulator.cc`

核心闸门：

- `min_length_2d`：短线直接跳过
- `line_tri_angle_threshold`：退化视角（近平面）剔除
- `IoU_threshold`：弱极线一致性剔除
- `sensitivity_threshold`：不稳定三角化剔除
- `SetRanges(ranges)`：超出场景范围候选剔除

可选增强：

- `use_vp`：VP 方向先验压制歧义
- `use_pointsfm`：点-线二部图/3D 点约束候选

### 6.5 全局打分与轨迹建图：再次压错误连接

位置：`src/limap/triangulation/global_line_triangulator.cc`

1. 跨视图支持打分
  - 使用 `min(score3d, score2d)` 累积支持，单图偶然误配难以高分通过
2. 候选连接上限与分数阈值
  - `fullscore_th`、`max_valid_conns`
3. 外边数量迭代剪枝
  - `min_num_outer_edges` + 迭代传播删除，清理孤立/弱连接节点
4. 2D/3D Linker 联合规则
  - 位置：`src/limap/base/line_linker.cc`
  - 规则：角度、重叠、smart-angle、垂距、innerseg、scale-inv 等联合判定

### 6.6 轨迹后处理：重投影与重叠一致性过滤

位置：`src/limap/runners/line_triangulation.py` + `src/limap/merging/merging_utils.cc`

1. `filter_tracks_by_reprojection`
  - 阈值：`th_angular_2d`、`th_perp_2d`
  - 动作：删除不支持当前 3D 线的 2D 观测，再重聚合
2. `remerge` + 再过滤
  - 迭代重合并后再次重投影过滤，减少重复 track 与错误合并
3. `filter_tracks_by_sensitivity`
  - 阈值：`th_sv_angular_3d`、`th_sv_num_supports`
  - 动作：去掉几何不稳定 track
4. `filter_tracks_by_overlap`
  - 阈值：`th_overlap`、`th_overlap_num_supports`
  - 动作：去掉投影重叠不足的伪 track
5. 最终可见视角门槛
  - 参数：`n_visible_views`
  - 动作：抑制偶然匹配支撑的线

### 6.7 BA/优化阶段：鲁棒损失与弱约束冻结

位置：`src/limap/optimize/line_refinement/refinement_config.h`、`src/limap/optimize/hybrid_bundle_adjustment/hybrid_bundle_adjustment.cc`

1. 鲁棒损失
  - 线几何默认 `CauchyLoss(0.25)`，VP/热图/特征项有独立 loss
  - 作用：减少离群观测牵引
2. 支持度不足不优化
  - 参数：`min_num_images`
  - 作用：观测不足线保持常量，避免被噪声“优化坏”
3. 相机/内参可设常量
  - `constant_intrinsics`、`constant_pose` 等
  - 作用：避免弱约束过拟合
4. 输出线段做 outlier-trim 聚合
  - `num_outliers_aggregator` 截断端点投影极值，减小少量错误支持影响

### 6.8 定位分支（补充）

位置：`src/limap/optimize/hybrid_localization/functions.py` + `estimators`

- `epipolar_filter` / `IoU_threshold`：先做极线一致性过滤
- `reprojection_filter_matches_2to3`：再按重投影距离和角度筛选
- 最后 `RANSAC / Hybrid RANSAC`（点线双阈值）估计位姿并系统性剔除离群

### 6.9 Fit&Merge 分支（补充）

位置：`src/limap/fitting/fitting.py`

- 深度/点云拟合时，对每条 2D 线做 3D 点集 RANSAC
- 关键参数：`ransac_th`、`min_percentage_inliers`
- 内点率低的候选直接丢弃，不进入后续 track 构建

### 6.10 纹理密集场景的优先调参顺序

1. 先减重复检测
  - 打开/加强 `line2d.do_merge_lines`
  - 降低 `line2d.max_num_2d_segs`
  - 提高 detector 置信阈值
2. 再收紧三角化几何门槛
  - 提高 `triangulation.IoU_threshold`
  - 提高 `triangulation.line_tri_angle_threshold`
  - 降低 `triangulation.sensitivity_threshold`（更严格）
3. 最后收紧 track 过滤
  - 降低 `filtering2d.th_angular_2d`、`filtering2d.th_perp_2d`
  - 提高 `n_visible_views`

这三步通常比直接换 matcher 更快见效。
