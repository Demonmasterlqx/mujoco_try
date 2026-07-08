# 双倒立摆控制器

Demo：

- `tutorial_10_double_inverted_pendulum`
- `tutorial_14_visualized_double_pendulum`

模型：`tutorial/models/double_inverted_pendulum.xml`

目标是做一个小型固定时间步系统：每个 MuJoCo timestep 抽取关节状态，并把它送进一个类似 `ros2_control` 的控制器框架。

## 模型

MJCF 模型包含：

- 两个 hinge joint：`joint1`、`joint2`
- 两个 motor actuator：`joint1_motor`、`joint2_motor`
- joint position、joint velocity、actuator force、tip position、simulation clock 传感器
- 一个 `small_error` keyframe，用作带小扰动的初始状态

MuJoCo timestep 固定写在 XML 中：

```xml
<option timestep="0.001" gravity="0 0 -9.81" integrator="Euler"/>
```

## 控制器结构

`mujoco_tutorial/double_pendulum_controller.hpp` 中的共享控制器代码把仿真拆成三块：

- `MujocoJointSystem::read()` 从 `mjData.qpos` 和 `mjData.qvel` 抽取关节位置、速度。
- `UprightPdController::update()` 根据状态和 period 计算 effort command。
- `MujocoJointSystem::write()` 把 effort 写入 `mjData.ctrl`。

固定时间步循环：

```cpp
mj_step1(model, data);
JointState state = system.read();
controller.update(data->time, fixed_period, state, command);
system.write(command);
mj_step2(model, data);
```

这模拟了 controller-manager 风格系统中的 `read -> update -> write` 生命周期，同时让示例不依赖 ROS 2。

## 运行

```bash
build/bin/tutorial_10_double_inverted_pendulum
```

最近一次本地验证信号：

- `fixed_period=0.001`
- `steps=3000` 且 `final_time=3`
- 最终关节位置收敛到接近 0
- 传感器表会输出两个关节的位置、速度、actuator force、末端位置和 clock

## 可视化运行

`tutorial_14_visualized_double_pendulum` 复用同一套 `MujocoJointSystem` 和 `UprightPdController`，但会在控制器推进仿真的同时，用 MuJoCo 渲染 API 打开 GLFW/OpenGL 窗口：

- `mjv_updateScene`
- `mjr_render`
- `mjr_overlay`
- 围绕 `mjv_moveCamera` 的 GLFW 键盘和鼠标回调

打开真实窗口：

```bash
build/bin/tutorial_14_visualized_double_pendulum --duration 8 --require-window
```

`--duration` 必须是有限且非负的秒数。使用 `--interactive` 可以忽略自动时长，让窗口一直运行到被关闭。

常用控制：

- Space：暂停或继续。
- Backspace：重置到 `small_error` keyframe，并重新激活控制器。
- Esc：关闭窗口。
- 鼠标左/右/中键拖拽和滚轮：旋转、平移、缩放相机。

批量或 CI 验证可以不打开窗口：

```bash
build/bin/tutorial_14_visualized_double_pendulum --headless-check
```

最近一次有显示环境的本地可视化验证：

```text
window_opened=true
duration=0.05
window_closed=true
final_time=0.05
simulation_steps=50
render_frames=3
```

## Debug 记录

来自 `debug.zh-CN.md` 的相关条目：

- 公共模型路径宏位于 `mujoco_tutorial_common`，因为加载 helper 被这个控制器 demo 和其他 demo 共用。
- 这里有意使用 `mj_step1`/`mj_step2`，让控制器在 MuJoCo 刷新位置和速度相关派生量之后再读取状态。
- 循环使用整数步数，而不是 `while (data->time < duration)`，避免边界处多跑一步。
- 共享控制器类型位于 `mujoco_tutorial` namespace；可执行 demo 应使用带 namespace 的名字，不要依赖旧的 anonymous-namespace 本地定义。
- 可视化 demo 在自动 `--duration` 运行时会限制总 MuJoCo 固定步数。渲染帧可以合并多个仿真 step，但不能在结尾额外多走一个 timestep。
- `--duration` 会严格解析：`0.05abc`、`nan`、`inf` 这类部分合法或非有限值会被拒绝。
- GLFW 初始化、窗口销毁和终止通过小型 RAII wrapper 成对管理，包含无法创建窗口而进入 fallback 的路径。
