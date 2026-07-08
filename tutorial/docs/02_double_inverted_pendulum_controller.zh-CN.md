# 双倒立摆控制器

Demo：`tutorial_10_double_inverted_pendulum`

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

C++ 代码把仿真拆成三块：

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

## Debug 记录

来自 `debug.zh-CN.md` 的相关条目：

- 公共模型路径宏位于 `mujoco_tutorial_common`，因为加载 helper 被这个控制器 demo 和其他 demo 共用。
- 这里有意使用 `mj_step1`/`mj_step2`，让控制器在 MuJoCo 刷新位置和速度相关派生量之后再读取状态。
- 循环使用整数步数，而不是 `while (data->time < duration)`，避免边界处多跑一步。
