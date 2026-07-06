# Double Inverted Pendulum Controller

Demo: `tutorial_10_double_inverted_pendulum`

Model: `tutorial/models/double_inverted_pendulum.xml`

The goal is a small fixed-step system that extracts joint state every MuJoCo timestep and passes it through a controller framework similar to `ros2_control`.

## Model

The MJCF model contains:

- two hinge joints: `joint1`, `joint2`
- two motor actuators: `joint1_motor`, `joint2_motor`
- sensors for joint position, joint velocity, actuator force, tip position, and simulation clock
- a keyframe `small_error` used as the initial perturbed state

The MuJoCo timestep is fixed in XML:

```xml
<option timestep="0.001" gravity="0 0 -9.81" integrator="Euler"/>
```

## Controller Structure

The C++ code separates the simulation into three pieces:

- `MujocoJointSystem::read()` extracts joint positions and velocities from `mjData.qpos` and `mjData.qvel`.
- `UprightPdController::update()` computes effort commands from state and period.
- `MujocoJointSystem::write()` writes efforts into `mjData.ctrl`.

The fixed-step loop is:

```cpp
mj_step1(model, data);
JointState state = system.read();
controller.update(data->time, fixed_period, state, command);
system.write(command);
mj_step2(model, data);
```

This mirrors the `read -> update -> write` lifecycle used by controller-manager style systems while keeping the example independent of ROS 2.

## Run

```bash
build/bin/tutorial_10_double_inverted_pendulum
```

Latest local verification signal:

- `fixed_period=0.001`
- `steps=3000` and `final_time=3`
- final joint positions converge close to zero
- sensor table reports the two joint positions, velocities, actuator forces, tip position, and clock

## Debug Notes

Relevant entries from `debug.md`:

- Public model path macros live on `mujoco_tutorial_common`, because the loader helpers are shared by this controller demo and all other demos.
- `mj_step1`/`mj_step2` is used deliberately so the controller reads state after MuJoCo has refreshed position and velocity dependent quantities.
- The loop uses an integer step count instead of `while (data->time < duration)` to avoid one extra boundary step.
