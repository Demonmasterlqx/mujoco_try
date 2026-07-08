# MuJoCo 教程 Debug 记录

本文记录构建教程和导入模型时遇到的实现坑点。每条也同步反映在相关教程文档中。

## 2026-07-04 - 公共路径宏必须对教程 Demo 可见

- 现象：`cmake --build build -j 8` 在编译 `tutorial/src/common.cpp` 时失败，因为 `TUTORIAL_MODEL_DIR` 和 `TUTORIAL_RM26_DIR` 未声明。
- 原因：路径宏最初只定义在各个 demo executable 上，但 `model_path()` 和 `rm26_path()` 位于 `mujoco_tutorial_common`。plugin demo 还需要内置 MuJoCo root 路径。
- 修复：在 `tutorial/CMakeLists.txt` 中把 `TUTORIAL_MODEL_DIR`、`TUTORIAL_RM26_DIR` 和 `TUTORIAL_MUJOCO_ROOT` 移到 `mujoco_tutorial_common`，并设置为 public 可见。
- 影响文档：构建和编程基础章节说明路径宏属于 common tutorial target。

## 2026-07-04 - 原始 RM26 URDF 使用 ROS package:// Mesh 路径

- 现象：MuJoCo 的 `compile` sample 无法编译原始 RM26 URDF，并返回 `Could not compile model`。
- 原因：URDF 以 `package://rm26_version3_engineering_model/meshes/*.STL` 引用 mesh。MuJoCo 默认不会解析 ROS package URL。
- 修复：把 URDF 和 mesh 复制到 `tutorial/rm26_version3_engineering_model/`，保留原始 URDF，并生成可被 MuJoCo 加载的 URDF 变体，把路径改成 `../meshes/*.STL`。
- 影响文档：RM26 导入指南解释了原始失败和路径重写。

## 2026-07-04 - RM26 STL 文件可能超过 MuJoCo STL Decoder 面数限制

- 现象：路径重写后的 RM26 URDF 仍因 `decoder failed for mesh file ... base_link.STL` 失败；详细错误说明 STL faces 数量必须在 1 到 200000 之间。
- 原因：`base_link.STL` 有 725344 个 faces，`j4_link.STL` 有 245928 个 faces。MuJoCo STL decoder 会拒绝超过 200000 faces 的单个 STL。
- 修复：只拆分超限的 binary STL 文件，拆分时不丢弃三角面。两个 MuJoCo import URDF 现在都引用拆分后的 `base_link` 和 `j4_link` mesh。
- 影响文档：RM26 导入指南说明为什么需要 split mesh 引用。

## 2026-07-04 - RM26 end_link 需要正的 Inertial 数据

- 现象：split RM26 URDF 因 `mass and inertia of moving bodies must be larger than mjMINVAL` 在 `end_link` 上失败。
- 原因：`end_link` 通过 revolute joint 连接，是运动 body，但 SolidWorks URDF 导出中没有 inertial、visual 或 collision 数据。
- 修复：在 MuJoCo import URDF 变体中为 `end_link` 添加一个小的正 inertial block。
- 影响文档：RM26 导入指南说明空的运动 link 需要物理有效的 inertial 数据。

## 2026-07-04 - 逆动力学需要在 mj_forward 之后设置 qacc

- 现象：动力学 demo 打印出 0 的 `qfrc_inverse`，但它本来想查询非零加速度下的逆动力学。
- 原因：`data->qacc[0]` 在 `mj_forward` 前设置；`mj_forward` 作为前向动力学的一部分覆盖了 acceleration。
- 修复：在 `mj_inverse` 前立刻设置 `data->qacc[0]`。
- 影响文档：编程基础指南预期逆动力学输出为非零。

## 2026-07-04 - Callback 恢复应与之前的 Callback 比较

- 现象：callback demo 最初通过和 `nullptr` 比较来报告恢复是否成功。
- 原因：MuJoCo callback 是进程级全局状态；其他应用或测试可能已经安装了非空 callback。
- 修复：安装 demo callback 前保存旧 callback，用 RAII guard 恢复，并和保存的指针比较。
- 影响文档：callback 章节描述 scoped restoration，而不是假设 callback 初始为 null。

## 2026-07-04 - MuJoCo 3.10 的 mj_fullM 签名和旧示例不同

- 现象：`tutorial_06_dynamics_queries` 编译失败，因为 `mj_fullM(model, mass_matrix.data(), data->qM)` 无法把第二个参数转换为 `const mjData*`。
- 原因：本地 MuJoCo 3.10 header 声明的是 `mj_fullM(const mjModel* m, const mjData* d, mjtNum* dst)`。
- 修复：把调用改成 `mj_fullM(model, data, mass_matrix.data())`。
- 影响文档：动力学教程使用本地 3.10 签名。

## 2026-07-04 - mjSpec 字符串字段优先使用 mjs_setString

- 现象：`tutorial_07_model_editing` 在直接写入 model name 时 segfault。
- 原因：虽然 C++ header 中 `mjString` 映射为 `std::string`，但可变长度字段的稳定 API 是 `mjs_setString()`。跨本地二进制/header 细节时，直接写入更容易出错。
- 修复：把 model name 赋值改为 `mjs_setString(spec->modelname, "programmatic_box")`。
- 影响文档：模型编辑教程使用 setter API 处理可变长度字段。

## 2026-07-04 - mj_saveXMLString 返回值不是 Boolean

- 现象：`tutorial_07_model_editing` 编译出了程序化模型，但在 `mj_saveXMLString` 处报告失败，且没有详细错误文本。
- 原因：`mj_saveXMLString` 成功返回 `0`，失败返回 `-1`，buffer 太小时返回所需输出大小。把返回值当 boolean 会反转成功情况。
- 修复：检查 `save_result != 0`，并在失败信息里包含返回码。buffer 仍保持足够大。
- 影响文档：模型编辑教程明确指出该返回约定不是 boolean。

## 2026-07-04 - 固定步长循环应使用整数步数

- 现象：双倒立摆 demo 最初跑到 `3.001` 秒，虽然目标时长是 3 秒。
- 原因：初版使用 `while (data->time < 3.0)`。浮点时间和 MuJoCo 的 post-step 时间更新会在边界处多走一步。
- 修复：共享步进逻辑现在使用 `step_count_for_duration()` 和整数循环；双倒立摆控制器在 `0.001` 秒 timestep 下运行 3000 次。
- 影响文档：双倒立摆指南预期 `steps=3000` 和 `final_time=3`。

## 2026-07-04 - mjuiDef Shortcut 字符串会被严格解析

- 现象：`tutorial_12_ui_definition` 运行时报 `mjui_add: invalid shortcut specification`。
- 原因：demo 在 section/check item 的 `other` 字段里填了随意的 shortcut 字符串。MuJoCo 原生 UI parser 期望特定的 shortcut 格式。
- 修复：从 headless UI definition demo 中移除 shortcut 字符串，只在 slider range 这类必需场景中保留 `other`。
- 影响文档：编程基础指南把 UI demo 定位为结构定义示例，而不是 shortcut parser 示例。

## 2026-07-04 - Rangefinder 方向是 Site 的正 Z 轴

- 现象：传感器 demo 中的 `down_range` 报 `-1`，表示没有 geom 交点。
- 原因：绑定到 site 的 rangefinder ray 使用 site 的正 Z 轴，并且会排除与传感器 site 绑定在同一 body 上的 geom。早期版本要么朝向空处，要么把 target geom 放在 `worldbody`，也就是和 site 相同的 body 上。
- 修复：沿 site 正 Z 方向在一个 child body 中添加 `range_target` geom，并把传感器重命名为 `target_range`。
- 影响文档：传感器套件指南说明 `target_range` 应产生正的命中距离。

## 2026-07-08 - 共享控制器重构需要带 Namespace 的名字

- 现象：把双倒立摆控制器移动到共享教程文件后，`tutorial_10_double_inverted_pendulum` 编译失败，因为 `MujocoJointSystem`、`UprightPdController` 和 `JointState` 不再位于该 executable 的本地 namespace。
- 原因：第一版可视化 demo 重构把可复用控制器类型放进了 `mujoco_tutorial` namespace，但原 executable 仍使用旧 anonymous-namespace 实现中的非限定名字。
- 修复：把 executable 更新为使用 `mujoco_tutorial::MujocoJointSystem`、`mujoco_tutorial::UprightPdController` 和 `mujoco_tutorial::JointState`。
- 影响文档：双倒立摆指南说明控制器现在是共享且带 namespace 的。

## 2026-07-08 - 可视化 Duration 应限制总固定步数

- 现象：`tutorial_14_visualized_double_pendulum --duration 0.05` 最初结束在 `final_time=0.051`。
- 原因：第一版渲染循环每帧推进一个近似 1/60 秒的完整仿真块。MuJoCo timestep 为 0.001 秒时，这个块会取整到 17 个仿真 step，因此自动 duration 边界可能被跨过一个或多个 timestep。
- 修复：用 `step_count_for_duration()` 计算整数 `steps_per_frame` 和整数 `remaining_steps`，并把每个渲染帧限制在剩余步数预算内。相同命令现在输出 `simulation_steps=50` 和 `final_time=0.05`。
- 影响文档：双倒立摆指南说明渲染帧可以合并多个仿真 step，但自动运行仍会限制总固定步数。

## 2026-07-08 - CMake 探测不要在源码根目录运行

- 现象：一次探索性的 `cmake --find-package` 检查在仓库顶层留下了未跟踪的 `CMakeFiles/` 目录。
- 原因：CMake 的一次性 find-package 模式从源码根目录运行时，仍会相对当前工作目录写入临时文件。
- 修复：删除生成目录，并把后续验证放到明确的构建目录中，例如 `build/` 或 `/tmp/...`。
- 影响文档：overview 构建章节提示探索性的 CMake 探测不要从仓库根目录发起。

## 2026-07-08 - 可视化改变了默认文本模式假设

- 现象：添加 `tutorial_14_visualized_double_pendulum` 后，overview 仍说教程默认是文本模式。
- 原因：这个说法对前 13 个 demo 成立，但新的可视化 demo 默认会打开 GLFW/OpenGL 窗口，除非显式选择 `--headless-check`。
- 修复：把 overview 改为“多数 demo 是文本模式”，并单独说明可视化 demo 的窗口模式和 headless 模式。
- 影响文档：overview 记录了这个例外。

## 2026-07-08 - 可视化新增 GLFW 和 OpenGL 构建依赖

- 现象：代码审查发现顶层 `find_package(OpenGL REQUIRED)` 和 `find_package(glfw3 REQUIRED)` 会阻塞只需要已有 headless demos 的用户配置工程。
- 原因：第一版 CMake 改动把可视化依赖变成了整个 tutorial 子项目的全局依赖。
- 修复：把可视化 demo 依赖改为 `QUIET`，只有找到 GLFW/OpenGL 时才构建 `tutorial_14_visualized_double_pendulum`，并让 `tutorial_01` 到 `tutorial_13` 在缺少这些依赖时仍可构建。
- 影响文档：README 和 overview 现在说明 CMake 能找到 `glfw3` 和 `OpenGL` 时会构建窗口 demo，否则只跳过该 demo。

## 2026-07-08 - Duration CLI 解析必须严格

- 现象：代码审查发现 `--duration 0.05abc`、`--duration nan` 和 `--duration inf` 可能被基于 `std::stod` 的解析接受。
- 原因：第一版 parser 没检查 `std::stod` 消耗了多少字符，也没有拒绝非有限值。
- 修复：接受 `--duration` 前检查整串都被消费、数值有限、且非负。
- 影响文档：双倒立摆指南说明 `--duration` 必须有限且非负。

## 2026-07-08 - GLFW 生命周期清理需要成对

- 现象：代码审查发现 `glfwInit()` 成功但 `glfwCreateWindow()` 失败时，会直接进入 fallback，没有显式 `glfwTerminate()`。
- 原因：第一版实现沿用了 MuJoCo 短 sample 的风格，只手动清理了正常路径中的窗口。
- 修复：增加小型 RAII wrapper，让已初始化的 GLFW session 和已创建的 window 在所有返回路径上都能成对清理。
- 影响文档：双倒立摆指南记录了可视化 demo 的 GLFW 生命周期规则。
