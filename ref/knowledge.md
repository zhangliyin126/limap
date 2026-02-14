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

## 5. Associate / Merging / BA / Localization 模块梳理

这一章补齐第 4 章后半段没展开的“四个后端模块”：`merging`、`global_pl_association`、`hybrid_bundle_adjustment`、`hybrid_localization`。写法保持和 6.3 一样，按“入口 -> 核心实现 -> 参数与边界”展开。

### 5.1 模块边界与入口（先看调用关系）

1. `merging`（3D 线段成轨与后过滤）
  - 代码：
    - `src/limap/merging/merging.cc`
    - `src/limap/merging/aggregator.cc`
    - `src/limap/merging/merging_utils.cc`
    - `src/limap/merging/merging.py`
  - 入口：
    - Fit&Merge 主流程：`src/limap/runners/line_fitnmerge.py`
    - Triangulation 后处理：`src/limap/runners/line_triangulation.py`（`remerge/filter_*`）
2. `global_pl_association`（点-线 / VP-线全局关联 + 联合优化）
  - 代码：
    - `src/limap/optimize/global_pl_association/global_associator.cc`
    - `src/limap/optimize/global_pl_association/cost_functions.h`
  - 入口：
    - 主脚本：`runners/pointline_association.py`
    - Python 封装：`src/limap/optimize/global_pl_association/__init__.py`
3. `hybrid_bundle_adjustment`（点线联合 BA 核心引擎）
  - 代码：
    - `src/limap/optimize/hybrid_bundle_adjustment/hybrid_bundle_adjustment.cc`
    - `src/limap/optimize/hybrid_bundle_adjustment/hybrid_bundle_adjustment_config.h`
  - 入口：
    - `src/limap/optimize/hybrid_bundle_adjustment/solve.py`
    - 在 runner 中常见调用：`optimize.solve_line_bundle_adjustment(...)`
4. `hybrid_localization`（线/点线定位：匹配过滤 + Ceres 位姿优化）
  - 代码：
    - `src/limap/optimize/hybrid_localization/hybrid_localization.cc`
    - `src/limap/optimize/hybrid_localization/cost_functions.h`
    - `src/limap/optimize/hybrid_localization/functions.py`
  - 入口：
    - 定位 runner：`src/limap/runners/hybrid_localization.py`
    - 姿态估计桥接：`src/limap/estimators/absolute_pose/_pl_estimate_absolute_pose.py`
    - C++ 估计器：`src/limap/estimators/absolute_pose/joint_pose_estimator.cc`

### 5.2 Merging 模块：从离散 3D 线段到 LineTrack

#### 5.2.1 输入输出与数据组织

1. 输入（`MergeToLineTracks`）
  - `all_lines_2d`: `{img_id: [Line2d]}`
  - `all_lines_3d`: `{img_id: [Line3d]}`
  - `neighbors`: `{img_id: [neighbor_img_id]}`
  - `linker`: `LineLinker(2D + 3D 规则)`
  - `imagecols`: 相机视图，用于跨图重投影检查
2. 输出
  - `Graph`：节点是 `(image_id, line_id)`，边是可合并关系
  - `linetracks`：每个 track 包含 `line2d_list/line3d_list/score_list`，并聚合出一条最终 `track.line`
3. 不确定度注入（Python 侧）
  - `merging.merging(...)` 在进 C++ 前，会调用 `_SetUncertaintySegs3d`
  - 每条 3D 线先按 `view + var2d` 估计 `uncertainty`，后续 3D linker 会用到

#### 5.2.2 `MergeToLineTracks` 主流程（`merging.cc`）

1. 初始化
  - 强制把 `linker.linker_3d.config` 切到 `set_to_spatial_merging()`
  - 只给 `length>0` 的 3D 线建图节点
2. 候选边生成（同图 + 跨图）
  - 同图：
    - 两两线段先过 `linker.check_connection_3d(l1,l2)`
    - 再过 `linker.check_connection_2d(line2d_i, line2d_j)`
  - 跨图：
    - 对 `image_id` 和 `neighbors[image_id]` 做候选
    - 用奇偶 `key` 规避 OpenMP 重复配对
    - 约束为“3D 可连 + 双向重投影 2D 可连”：
      - `l1` 投影到邻图，与邻图 2D 线检查
      - `l2` 投影回当前图，再检查一次
3. 图边打分与入图
  - `score = length(line1) + length(line2)`
  - `graph.AddEdge(node1, node2, score)`
4. 成轨（默认 Greedy）
  - 直接调用 `ComputeLineTrackLabelsGreedy(...)`
  - 然后按标签把节点的 `line2d/line3d/score` 填入对应 track
5. 轨迹线聚合
  - 每个 track 用 `Aggregator::aggregate_line3d_list(...)` 输出最终 `track.line`

#### 5.2.3 三种 track-label 合并策略（`merging.h/.cc`）

1. `ComputeLineTrackLabelsGreedy`
  - 边按相似度降序做并查集合并
  - 合并启发式按集合大小（`images_in_track.size()`）做 parent 选择
  - 特点：快，默认路径使用它
2. `ComputeLineTrackLabelsExhaustive`
  - 先切到 `linker3d.config.set_to_avgtest_merging()`
  - 合并两个集合前，遍历两集合中“有重叠”的线对，逐一 `check_connection`
  - 特点：最严格，复杂度高
3. `ComputeLineTrackLabelsAvg`
  - 同样 `set_to_avgtest_merging()`
  - 用每个集合的“均值线”先验做连通判定
  - 特点：比 exhaustive 快，比 greedy 更保守

#### 5.2.4 线聚合器 `Aggregator`（`aggregator.cc`）

1. `n_lines < 4`
  - 走 `takebest`（按 score 取最优）
  - 同时把输出线 `uncertainty` 设为输入最小不确定度
2. `n_lines >= 4`
  - 所有端点做 TLS/SVD 主方向估计
  - 把端点投影到主方向后排序
  - 用 `num_outliers` 做两端截断，重建 `start/end`
3. 工程含义
  - 支持线多时，端点由“投影分位数”控制，鲁棒性强于简单均值

#### 5.2.5 `RemergeLineTracks`（迭代重合并）

1. 先按当前 `track.line` 两两建边
  - 规则：`linker3d.check_connection(l1,l2)`
  - 仅 `active=true` 的轨迹作为主遍历集合（减少开销）
2. 并查集求连通分组
  - 同组轨迹把 `node/image/line2d/line3d/score` 全部拼接
3. 组内再聚合
  - 再次 `Aggregator::aggregate_line3d_list(...)`
  - 若某组只含 1 条原轨迹，则新轨迹 `active=false`
4. Python 包装 `merging.remerge(...)`
  - 会循环调用 `_RemergeLineTracks`，直到轨迹数不再减少

#### 5.2.6 后过滤链（`merging_utils.cc`）

1. `filter_tracks_by_reprojection`
  - 对每条 support 检查：
    - 角度误差 `angle <= th_angular_2d`
    - 端点垂距 `dist_endperp <= th_perp_2d`
  - 只保留通过的 support，再重聚合 3D 线
2. `filter_tracks_by_sensitivity`
  - 用 `track.line.sensitivity(view)` 判定每个 support
  - 至少 `min_num_supports` 个不同图像通过才保留 track
3. `filter_tracks_by_overlap`
  - 比较 `track.line` 投影与 support 2D 线的 overlap
  - 通过图像数达到 `min_num_supports` 才保留

#### 5.2.7 在流程里的落点

1. `line_fitnmerge.py`
  - 先 `merging.merging(...)` 成轨
  - 再 `filter_tracks_by_reprojection` -> `remerge` -> 再过滤
2. `line_triangulation.py`
  - 三角化输出 tracks 后，直接走同样的 `filter/remerge/filter` 后处理链

### 5.3 Associate 模块：`GlobalAssociator`（点线 + VP 结构约束）

#### 5.3.1 模块定位与输入

1. 类继承关系
  - `GlobalAssociator : HybridBAEngine`
  - 继承了相机/点/线的 BA 几何残差与参数化
2. 新增输入
  - 2D 点线二部图：`all_bpt2ds_`（`PL_Bipartite2d`）
  - 2D VP线二部图：`all_bpt2ds_vp_`（`VPLine_Bipartite2d`，可选）
  - VP tracks / VP directions（可选）
3. 典型入口（runner）
  - `runners/pointline_association.py`：
    - `InitImagecols / InitPointTracks / InitLineTracks`
    - `Init2DBipartites_PointLine`
    - 可选 `InitVPTracks + Init2DBipartites_VPLine`
    - `SetUp()` -> `Solve()`

#### 5.3.2 `SetUp` 的残差拼装（在 BA 基础上加结构项）

1. 继承残差（R1）
  - 点几何重投影残差（R1.1）
  - 线几何重投影残差（R1.2）
2. 新增结构残差（R2）
  - R2.1 点-线 3D 关联残差：`PointLineAssociation3dFunctor`
  - R2.2 VP-线 3D 关联残差：`VPLineAssociation3dFunctor`
  - R2.3 VP 正交残差：`VPOrthogonalityFunctor`
  - R2.4 VP 共线残差：`VPCollinearityFunctor`
3. 参数化
  - 相机/点/线：沿用 `HybridBAEngine`
  - VP：`constant_vp=true` 时固定，否则 `SetSphereManifold<3>`

#### 5.3.3 软关联权重怎么构造

1. 点-线权重 `construct_weights_pointline`
  - 遍历每条 line track 的每个 support
  - 从 `all_bpt2ds_[img_id]` 拿该 2D 线的邻接点
  - 点必须有 `point3D_id >= 0`
  - 距离用“2D 点到 2D 线投影的垂距”
  - `dist2weight` 当前是硬阈值：
    - `dist <= th_pixel` 记 `1`
    - 否则记 `0`
  - 同一 `(point3d_id, line3d_id)` 在多图上求和，达到 `th_weight_pointline` 才保留
2. VP-线权重 `construct_weights_vpline`
  - 统计每条 line 在 supports 上对应到各 VP 的次数
  - 每条 line 只保留“计数最高”的 VP
  - 计数 `< th_count_vpline` 则丢弃
3. 残差缩放
  - 点-线：`weight * lw_pointline_association`
  - VP-线：`count * 1e2 * lw_vpline_association`
  - VP 正交/共线：`1e2 * lw_*`

#### 5.3.4 `ReassociateJunctions`（可选补点）

1. 从 2D bipartite 统计“同一 2D junction 连接到两条 3D line track”的证据
2. 对每对 track 统计跨图计数，计数需 `>= th_count_lineline`
3. 2D 与 3D 都要求角度足够大（`th_angle_lineline`）
4. 通过两条 3D 线的中点构造一个新 `PointTrack`
5. 回写对应 2D 点的 `point3D_id`
6. 注意
  - 该函数在 `runners/pointline_association.py` 默认是注释掉的（可人工开启）

#### 5.3.5 输出阶段：软约束到硬关联

1. `GetBipartite3d_PointLine_Constraints`
  - 仅按软权重保留边（不加 3D 距离硬门槛）
2. `GetBipartite3d_PointLine`
  - 在软权重基础上，再加硬约束：
    - `point-line 3D distance <= th_hard_pl_dist3d * point_uncertainty`
  - `point_uncertainty` 取该点在各视图中的最小相机不确定度
3. `GetBipartite3d_VPLine`
  - 先按软计数关联，再过硬角度门槛：
    - `angle(vp,line) <= th_hard_vpline_angle3d`
  - 最后删掉度为 0 的 VP 节点
4. 线段输出的稳健处理
  - 先从优化后的无限线恢复有限线段
  - 再做 `test_linetrack_validity`（至少半数 support 图像重投影有效）
  - 不通过时回退到优化前 track 线段

#### 5.3.6 配置分组（`GlobalAssociatorConfig`）

1. 变量冻结
  - `constant_intrinsics / constant_principal_point / constant_pose`
  - `constant_point / constant_line / constant_vp`
2. 结构权重
  - `lw_pointline_association`
  - `lw_vpline_association`
  - `lw_vp_orthogonality`
  - `lw_vp_collinearity`
3. 软权重阈值
  - `th_pixel`（源码字段名）
  - `th_weight_pointline`
  - `th_count_vpline`
4. 硬输出阈值
  - `th_hard_pl_dist3d`
  - `th_hard_vpline_angle3d`

### 5.4 BA 模块：`HybridBAEngine`（点线联合）

#### 5.4.1 变量表示与状态

1. 相机
  - 来自 `ImageCollection`，参数块为 `intrinsics + qvec + tvec`
2. 点
  - `points_`: `track_id -> V3D`
3. 线
  - `lines_`: `track_id -> MinimalInfiniteLine3d(uvec,wvec)`
  - `uvec` 用四元数表示旋转部分，`wvec` 是 2D 球面参数（正交表示）

#### 5.4.2 残差项（`hybrid_bundle_adjustment.cc`）

1. 点几何残差 `AddPointGeometricResiduals`
  - 对每个点观测加 2 维重投影误差
  - 按相机模型模板实例化 `PointGeometricRefinementFunctor`
  - 全局权重 `lw_point`
2. 线几何残差 `AddLineGeometricResiduals`
  - 对每个 support 2D 线加几何残差
  - cost 来自 `line_refinement::GeometricRefinementFunctor`
  - 每条 support 的权重由 `ComputeLineWeights(track)` 给出（默认与 2D 线长成正比，`length/30`）
3. `SetUp` 开关逻辑
  - 若对应变量全是常量，则该类残差不会被加入问题

#### 5.4.3 参数化与冻结策略

1. 相机参数
  - `constant_intrinsics=true`：内参全冻结
  - 否则若 `constant_principal_point=true`：只冻结主点参数（subset manifold）
  - `constant_pose=true`：`qvec/tvec` 全冻结，否则对 `qvec` 施加四元数流形
2. 点参数
  - `constant_point=true` 时冻结
3. 线参数
  - 若 `constant_line=true` 或 `track.count_images() < min_num_images`：冻结该线
  - 否则 `uvec` 上四元数流形，`wvec` 上 `SphereManifold<2>`
4. 实用快捷
  - `HybridBAConfig.set_constant_camera()` 一次性冻结内参与位姿（runner 常用）

#### 5.4.4 求解器与输出

1. 求解器选择（按图像数量）
  - `<= 50`: `DENSE_SCHUR`
  - `<= 900`: `SPARSE_SCHUR`
  - `> 900`: `ITERATIVE_SCHUR + SCHUR_JACOBI`
2. 输出线段恢复
  - 从优化后的无限线 + 原 track 支持线段恢复有限线段
  - 使用 `num_outliers` 截断端点投影极值（鲁棒端点）
3. 输出接口
  - `GetOutputImagecols / GetOutputPoints / GetOutputPointTracks`
  - `GetOutputLines / GetOutputLineTracks`

#### 5.4.5 和 runner 的接线方式

1. `line_triangulation.py` / `line_fitnmerge.py`
  - 都在后过滤后调用 `solve_line_bundle_adjustment(...)`
  - 再用 `GetOutputLineTracks(num_outliers_aggregator)` 替换轨迹几何
2. 与 `GlobalAssociator` 的关系
  - `GlobalAssociator` 直接复用这个引擎（几何项 + 参数化 + solver 选择）

### 5.5 定位模块：`hybrid_localization`（线、点线位姿优化）

#### 5.5.1 两层结构

1. 匹配与过滤层（Python）
  - `match_line_2to2_epipolarIoU`
  - `filter_line_2to2_epipolarIoU`
  - `match_line_2to3`
  - `reprojection_filter_matches_2to3`
2. 优化与估计层（C++ + Python 包装）
  - Ceres 引擎：`LineLocEngine` / `JointLocEngine`
  - RANSAC 估计器：`JointPoseEstimator` 与 Hybrid/RANSAC 包装

#### 5.5.2 2D->3D 线匹配构建（`functions.py`）

1. `match_line_2to2_epipolarIoU`
  - 参考图/目标图线段两两计算 epipolar IoU
  - `IoU > threshold` 才保留
2. `match_line_2to3`
  - 用 `line2track[tgt_img_id][tgt_line_id]` 把 2D-2D 对映射为 2D-3D track 对
3. `reprojection_filter_matches_2to3`
  - 对每条 query 2D 线，从候选 track 中挑损失最小者
  - 支持距离函数：
    - `Perpendicular`
    - `Midpoint`
    - `Midpoint_Perpendicular`
  - `Midpoint` 模式还会叠加方向差惩罚并用 `sine_thres` 过滤大角差

#### 5.5.3 `LineLocEngine` / `JointLocEngine` 优化细节

1. 变量
  - 内参 `kvec`（固定）
  - 外参 `qvec + tvec`（优化）
2. 线残差构建
  - 同一条 3D 线可对应多条 2D 线
  - 每条 2D 线残差按 `length / sum_lengths` 归一化加权
3. 点残差（`JointLocEngine`）
  - 额外加入点重投影（或点到视线 3D 距离）残差
  - 总权重由 `weight_line` 与 `weight_point` 控制
4. 求解器
  - 统一 `DENSE_QR`

#### 5.5.4 可选线残差与权重函数（`cost_functions.h`）

1. 线残差类型 `LineLocCostFunction`
  - 2D：`E2DMidpointDist2` / `E2DMidpointAngleDist3`
  - 2D：`E2DPerpendicularDist2` / `E2DPerpendicularDist4`
  - 3D：`E3DLineLineDist2` / `E3DPlaneLineDist2`
2. 残差权重类型 `LineLocCostFunctionWeight`
  - `ENoneWeight / ECosineWeight / ELine3dppWeight / ELengthWeight / EInvLengthWeight`
3. 点残差模式切换
  - 当线损失选 `E3DLineLineDist2` 或 `E3DPlaneLineDist2` 时，代码会开启 `points_3d_dist=true`
  - 这时点项使用“点到视线 3D 距离”而非 2D 重投影差

#### 5.5.5 与绝对位姿估计器的耦合

1. Python 入口 `_pl_estimate_absolute_pose(...)`
  - `ransac.method is None`：直接 `optimize.solve_jointloc(...)` 做纯优化
  - 否则走 `EstimateAbsolutePose_PointLine` / `EstimateAbsolutePose_PointLine_Hybrid`
2. C++ 最小求解器组合（`joint_pose_estimator.cc`）
  - `3点0线` -> `p3p`
  - `2点1线` -> `p2p1ll`
  - `1点2线` -> `p1p2ll`
  - `0点3线` -> `p3ll`
3. 非最小与最小二乘精化
  - 非最小样本和最终 LS 都会用 `JointLocEngine` 做局部优化
4. 内点评估
  - 点：重投影误差（或 3D 视线距离）
  - 线：`ReprojectionLineFunctor` 对应残差范数
  - 都带 cheirality 检查；线还额外检查重投影重叠长度

#### 5.5.6 `runners/hybrid_localization.py` 的落地时序

1. 先准备 query/db 线段与 `line2track` 映射
2. 根据 `localization.2d_matcher`：
  - `epipolar`：在线几何上直接筛配对
  - 其他 matcher：先提取/匹配，再可选 `epipolar_filter`
3. 线匹配转 2D->3D 后，可选 `reprojection_filter` 压成 1-1
4. 取 HLoc 点对应 + 线对应，调用 `estimators.pl_estimate_absolute_pose(...)`
5. 输出每个 query 的最终 `qvec + tvec`

### 5.6 四模块串联视角（工程上怎么连）

1. Triangulation 主线（最常见）
  - `triangulation` 产出 `linetracks`
  - `merging.filter/remerge` 做轨迹清洗
  - `hybrid_bundle_adjustment` 做几何精化
2. Fit&Merge 分线
  - 深度/点云先拟合单图 3D 线
  - `merging` 跨图成轨 + 后过滤
  - 可选 `hybrid_bundle_adjustment`
3. 结构关联分线
  - 读取已有 `pointtracks + linetracks + 2d bipartite`
  - `global_pl_association` 做点线/VP线联合优化与关联输出
4. 定位分线
  - query 与 db 建立线对应（可混点对应）
  - RANSAC（可选）+ `JointLocEngine` 精化
  - 输出 query 位姿

对这四块的实践调参顺序通常是：
1. 先稳 correspondence（2D-2D、2D-3D 过滤）
2. 再调 merging 阈值（角度/重叠/垂距）
3. 再放 BA（确认 `min_num_images`、常量开关）
4. 最后再加结构项（point-line / vp-line）和定位联合权重

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
