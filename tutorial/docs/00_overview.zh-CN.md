# MuJoCo C++ 教程总览

本教程参考的官方文档：

- MuJoCo Programming guide: https://mujoco.readthedocs.io/en/stable/programming/index.html
- Simulation: https://mujoco.readthedocs.io/en/stable/programming/simulation.html
- Visualization: https://mujoco.readthedocs.io/en/stable/programming/visualization.html
- Model Editing: https://mujoco.readthedocs.io/en/stable/programming/modeledit.html
- Code samples: https://mujoco.readthedocs.io/en/stable/programming/samples.html
- Extensions and plugins: https://mujoco.readthedocs.io/en/stable/programming/extension.html
- XML sensor reference: https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor

代码刻意保持小而直接，多数 demo 默认是文本模式。`tutorial_14_visualized_double_pendulum` 在有显示环境时会打开 GLFW/OpenGL 窗口，也支持用 `--headless-check` 做批量验证。依赖可用时，下面每个功能点都有一个可运行 demo，生成在 `build/bin/`；完整教程集合是 `tutorial_01` 到 `tutorial_14`。

## Demo 地图

| Demo | 源码 | 模型 | 目的 |
| --- | --- | --- | --- |
| `tutorial_01_load_and_step` | `tutorial/src/01_load_and_step.cpp` | `free_box.xml` | `mj_loadXML`、`mj_makeData`、`mj_step`、`qpos/qvel` |
| `tutorial_02_state_access` | `tutorial/src/02_state_access.cpp` | `hinge_arm.xml` | `mj_stateSize`、`mj_getState`、`mj_setState`、传感器 |
| `tutorial_03_control_loop` | `tutorial/src/03_control_loop.cpp` | `hinge_arm.xml` | 在 `mj_step` 前直接写入 `mjData.ctrl` |
| `tutorial_04_step_split_controller` | `tutorial/src/04_step_split_controller.cpp` | `hinge_arm.xml` | 使用 `mj_step1`/`mj_step2` 拆分控制流程 |
| `tutorial_05_callbacks` | `tutorial/src/05_callbacks.cpp` | `hinge_arm.xml` | 安装并恢复 `mjcb_control` 回调 |
| `tutorial_06_dynamics_queries` | `tutorial/src/06_dynamics_queries.cpp` | `hinge_arm.xml` | 质量矩阵、Jacobian、逆动力学、能量 |
| `tutorial_07_model_editing` | `tutorial/src/07_model_editing.cpp` | C++ 生成 | `mj_makeSpec`、`mjs_add*`、`mj_compile`、`mj_saveXMLString` |
| `tutorial_08_visualization_scene` | `tutorial/src/08_visualization_scene.cpp` | `hinge_arm.xml` | 不依赖 OpenGL 的 `mjvScene` 抽象可视化 |
| `tutorial_09_sensor_suite` | `tutorial/src/09_sensor_suite.cpp` | `sensor_suite.xml` | 内置传感器和 `mjcb_sensor` 用户传感器 |
| `tutorial_10_double_inverted_pendulum` | `tutorial/src/10_double_inverted_pendulum.cpp` | `double_inverted_pendulum.xml` | 固定时间步 read-update-write 控制循环 |
| `tutorial_11_rm26_import_check` | `tutorial/src/11_rm26_import_check.cpp` | RM26 split URDF | 验证导入的 RM26 模型能被 MuJoCo 加载 |
| `tutorial_12_ui_definition` | `tutorial/src/12_ui_definition.cpp` | 无 | 原生 `mjUI` section 和 item 定义 |
| `tutorial_13_plugin_loading` | `tutorial/src/13_plugin_loading.cpp` | MuJoCo plugin sample | 加载第一方插件库和插件传感器模型 |
| `tutorial_14_visualized_double_pendulum` | `tutorial/src/14_visualized_double_pendulum.cpp` | `double_inverted_pendulum.xml` | 在 GLFW/OpenGL 窗口中展示受控双倒立摆 |

## 构建与验证

可视化 demo 为教程构建新增了两个可选系统依赖：GLFW 和 OpenGL。当 CMake 能以 `glfw3` 和 `OpenGL` 解析它们时，会构建 `tutorial_14_visualized_double_pendulum`；否则只跳过这个窗口 demo，并继续构建 `tutorial_01` 到 `tutorial_13`。

```bash
cmake -S . -B build
cmake --build build -j 8
for exe in build/bin/tutorial_*; do
    case "$(basename "$exe")" in
        tutorial_14_visualized_double_pendulum) "$exe" --headless-check ;;
        *) "$exe" ;;
    esac
done
```

构建期路径宏被有意挂在公共教程目标上，并以 public 可见性传递：

- `TUTORIAL_MODEL_DIR`：多个 demo 共用的 XML 模型目录。
- `TUTORIAL_RM26_DIR`：复制进教程的 RM26 导入目录。
- `TUTORIAL_MUJOCO_ROOT`：内置 SDK 下的第一方插件示例路径。

除非公共 helper API 也随之调整，否则不要把这些宏下放到单个可执行文件。

探索性的 CMake 探测命令不要在源码根目录运行。一次性的 find-package 检查如果从仓库根目录发起，可能留下顶层 `CMakeFiles/` 目录；它不是项目的一部分。

最近一次本地验证中，14 个教程 demo 全部以 exit code 0 结束。关键输出检查：

- `tutorial_10_double_inverted_pendulum` 输出 `steps=3000`、`fixed_period=0.001` 和 `final_time=3`。
- `tutorial_14_visualized_double_pendulum --duration 0.05` 在有显示环境时输出 `window_opened=true`、`simulation_steps=50` 和 `final_time=0.05`。批量运行时用 `--headless-check`，会输出 `headless_check_steps=250`。
- `tutorial_09_sensor_suite` 输出正的 `target_range` 命中距离，当前是 `0.560000`。
- `tutorial_11_rm26_import_check` 输出：

```text
model=rm26_version3_engineering_model nq=13 nv=13 nu=0 nsensor=0 timestep=0.002
nbody=14 njnt=13 ngeom=18 nmesh=18
```

直接 MuJoCo RM26 URDF 和显式 split URDF 也能通过内置 `compile` sample：

```bash
mujoco/mujoco-3.10.0/bin/compile tutorial/rm26_version3_engineering_model/urdf/rm26_version3_engineering_model_mujoco.urdf
mujoco/mujoco-3.10.0/bin/compile tutorial/rm26_version3_engineering_model/urdf/rm26_version3_engineering_model_mujoco_split.urdf
```

## Debug 记录

所有实现过程中遇到的坑都记录在 `debug.zh-CN.md`。每个专题文档也会在触发该问题的功能附近重复对应注意事项。
