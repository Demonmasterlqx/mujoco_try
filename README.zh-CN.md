# MuJoCo C++ 教程工作区

本工作区提供一套循序渐进的 MuJoCo C++ 教程，使用仓库内置的 MuJoCo SDK：`mujoco/mujoco-3.10.0`。

英文原文保留在 `README.md`、`debug.md` 和 `tutorial/docs/*.md`；中文对照版使用 `*.zh-CN.md` 命名。

## 构建

当系统安装了 GLFW 和 OpenGL 开发包，并且 CMake 能以 `glfw3` 和 `OpenGL` 找到它们时，会构建可视化 demo。如果缺少这些依赖，CMake 会跳过 `tutorial_14_visualized_double_pendulum`，但仍会构建 headless demos。

```bash
cmake -S . -B build
cmake --build build -j 8
```

教程公共目标会把 `TUTORIAL_MODEL_DIR`、`TUTORIAL_RM26_DIR` 和 `TUTORIAL_MUJOCO_ROOT` 公开给每个 demo。请把这些路径宏保留在 `mujoco_tutorial_common` 上；如果挪到单个可执行文件，`model_path()`、`rm26_path()` 这类共享 helper 会找不到路径。

## 运行全部 Demo

GLFW/OpenGL 可用时，完整教程 demo 为 `tutorial_01` 到 `tutorial_14`。`tutorial_14_visualized_double_pendulum` 在有显示环境时会打开窗口；批量验证时使用 `--headless-check`。

```bash
for exe in build/bin/tutorial_*; do
    case "$(basename "$exe")" in
        tutorial_14_visualized_double_pendulum) "$exe" --headless-check ;;
        *) "$exe" ;;
    esac
done
```

如果要边仿真边看受控双倒立摆画面：

```bash
build/bin/tutorial_14_visualized_double_pendulum --duration 8 --require-window
```

Space 暂停，Backspace 重置，Esc 关闭窗口，鼠标用于移动相机。

## 文档

- `tutorial/docs/00_overview.zh-CN.md`：路线图和 demo 索引。
- `tutorial/docs/01_programming_basics.zh-CN.md`：加载、步进、状态、控制、回调、动力学查询、模型编辑、抽象可视化、原生 UI 和插件加载。
- `tutorial/docs/02_double_inverted_pendulum_controller.zh-CN.md`：固定时间步双关节倒立摆系统、类似 `ros2_control` 的控制器循环，以及 GLFW 可视化 demo。
- `tutorial/docs/03_sensor_suite.zh-CN.md`：内置传感器和用户自定义传感器 demo。
- `tutorial/docs/04_rm26_import.zh-CN.md`：RM26 URDF 导入、路径修复、mesh 拆分和验证。
- `debug.zh-CN.md`：实现和验证教程时遇到的坑点中文记录。

英文版本：

- `tutorial/docs/00_overview.md`
- `tutorial/docs/01_programming_basics.md`
- `tutorial/docs/02_double_inverted_pendulum_controller.md`
- `tutorial/docs/03_sensor_suite.md`
- `tutorial/docs/04_rm26_import.md`
- `debug.md`
