# RM26 模型导入

源模型：

```text
~/code/RM/RMController/core/models/rm26_version3_engineering_model
```

导入后的副本：

```text
tutorial/rm26_version3_engineering_model
```

验证 demo：

```bash
build/bin/tutorial_11_rm26_import_check
```

## 已复制文件

导入过程复制了 MuJoCo 运行时加载需要的 URDF 和 mesh：

```text
urdf/rm26_version3_engineering_model.urdf
meshes/*.STL
```

ROS package 文件、launch 文件、RViz 配置和导出日志不是 MuJoCo 运行时加载所必需的。

## 导入变体

当前保留三个 URDF 变体：

- `rm26_version3_engineering_model.urdf`：原始 SolidWorks/ROS URDF。
- `rm26_version3_engineering_model_mujoco.urdf`：直接 MuJoCo 导入 URDF，已把 `package://rm26_version3_engineering_model/meshes/` 改成 `../meshes/`，拆分过大的 mesh 引用，并为 `end_link` 添加有效 inertial 数据。
- `rm26_version3_engineering_model_mujoco_split.urdf`：等价的已验证导入入口，作为显式 split-mesh 变体保留。

demo 加载 split 版本。

## 验证结果

最近一次本地运行：

```text
model=rm26_version3_engineering_model nq=13 nv=13 nu=0 nsensor=0 timestep=0.002
nbody=14 njnt=13 ngeom=18 nmesh=18
first_body=image_pitch_link
time=0
```

这证明模型能导入本地 MuJoCo runtime，可以分配 `mjData` 并运行 `mj_forward`。

MuJoCo 的 `compile` sample 在不指定输出文件时，可以成功编译两个 MuJoCo import URDF：

```bash
mujoco/mujoco-3.10.0/bin/compile tutorial/rm26_version3_engineering_model/urdf/rm26_version3_engineering_model_mujoco.urdf
mujoco/mujoco-3.10.0/bin/compile tutorial/rm26_version3_engineering_model/urdf/rm26_version3_engineering_model_mujoco_split.urdf
```

原始 ROS URDF 在同一个工具中仍会按原样失败。第一个阻塞点是 ROS `package://` 路径；MuJoCo import URDF 还额外应用了 mesh 拆分和 `end_link` inertial 修复。

复现这个预期的原始 URDF 失败：

```bash
mujoco/mujoco-3.10.0/bin/compile tutorial/rm26_version3_engineering_model/urdf/rm26_version3_engineering_model.urdf
```

## Debug 记录

来自 `debug.zh-CN.md` 的相关条目：

- 原始 URDF 使用 ROS `package://` mesh 路径，MuJoCo 默认不会解析。
- `base_link.STL` 和 `j4_link.STL` 超过 MuJoCo STL decoder 的单文件 200000 faces 限制。它们已被拆成多个 binary STL 文件，没有丢弃三角面。
- `end_link` 是通过 revolute joint 连接的运动 body，但没有 inertial 数据。MuJoCo import URDF 为它添加了一个小的正 inertial block。

## 剩余建模注意事项

导入后的 RM26 模型是 passive 的：

- `nu=0`，因为 URDF 中没有 MuJoCo actuator。
- 部分源 joint limit 的 `effort="0"` 且 `velocity="0"`。
- 当前仍使用密集 CAD mesh 作为 visual/collision geometry。拆分修复能让加载成功，但生产仿真中应添加简化 collision geometry，以改善速度和接触质量。
