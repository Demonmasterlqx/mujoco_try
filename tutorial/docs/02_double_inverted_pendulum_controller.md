# Double Inverted Pendulum Controller

Demos:

- `tutorial_10_double_inverted_pendulum`
- `tutorial_14_visualized_double_pendulum`

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

The shared C++ controller code in `mujoco_tutorial/double_pendulum_controller.hpp` separates the simulation into three pieces:

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

## Visualized Run

`tutorial_14_visualized_double_pendulum` uses the same `MujocoJointSystem` and `UprightPdController`, but advances the controller while rendering a GLFW/OpenGL window with MuJoCo's rendering API:

- `mjv_updateScene`
- `mjr_render`
- `mjr_overlay`
- GLFW keyboard and mouse callbacks around `mjv_moveCamera`

Run with a real window:

```bash
build/bin/tutorial_14_visualized_double_pendulum --duration 8 --require-window
```

`--duration` must be a finite non-negative number of seconds. Use `--interactive` to ignore the automatic duration and keep the window running until it is closed.

Useful controls:

- Space: pause or resume.
- Backspace: reset to the `small_error` keyframe and reactivate the controller.
- Esc: close the window.
- Left/right/middle mouse drag and scroll: rotate, pan, and zoom the camera.

Batch or CI verification can avoid opening a window:

```bash
build/bin/tutorial_14_visualized_double_pendulum --headless-check
```

Latest local visual verification with a display:

```text
window_opened=true
duration=0.05
window_closed=true
final_time=0.05
simulation_steps=50
render_frames=3
```

## Debug Notes

Relevant entries from `debug.md`:

- Public model path macros live on `mujoco_tutorial_common`, because the loader helpers are shared by this controller demo and all other demos.
- `mj_step1`/`mj_step2` is used deliberately so the controller reads state after MuJoCo has refreshed position and velocity dependent quantities.
- The loop uses an integer step count instead of `while (data->time < duration)` to avoid one extra boundary step.
- The shared controller types live in the `mujoco_tutorial` namespace; executable demos should use qualified names instead of relying on the old anonymous-namespace local definitions.
- The visual demo caps the total number of fixed MuJoCo steps for automatic `--duration` runs. Rendering frames can group several simulation steps, but they must not add an extra timestep at the end.
- `--duration` is parsed strictly: partial values such as `0.05abc`, `nan`, and `inf` are rejected.
- GLFW initialization, window destruction, and termination are paired through small RAII wrappers, including fallback paths where a window cannot be created.
