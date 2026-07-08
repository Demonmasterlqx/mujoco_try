# MuJoCo C++ Tutorial Workspace

This workspace contains a step-by-step MuJoCo C++ tutorial built against the bundled MuJoCo SDK at `mujoco/mujoco-3.10.0`.

## Build

The visualized demo is built when system GLFW and OpenGL development packages are available to CMake as `glfw3` and `OpenGL`. If they are missing, CMake skips `tutorial_14_visualized_double_pendulum` and still builds the headless demos.

```bash
cmake -S . -B build
cmake --build build -j 8
```

The tutorial helper target publishes `TUTORIAL_MODEL_DIR`, `TUTORIAL_RM26_DIR`, and `TUTORIAL_MUJOCO_ROOT` to every demo. Keep those path macros on `mujoco_tutorial_common`; moving them to individual executables breaks shared helpers such as `model_path()` and `rm26_path()`.

## Run All Demos

The full tutorial set is `tutorial_01` through `tutorial_14` when GLFW/OpenGL are available. `tutorial_14_visualized_double_pendulum` opens a window when a display is available; use `--headless-check` for batch verification.

```bash
for exe in build/bin/tutorial_*; do
    case "$(basename "$exe")" in
        tutorial_14_visualized_double_pendulum) "$exe" --headless-check ;;
        *) "$exe" ;;
    esac
done
```

To see the controlled double inverted pendulum while it is simulated:

```bash
build/bin/tutorial_14_visualized_double_pendulum --duration 8 --require-window
```

Use Space to pause, Backspace to reset, Esc to close, and the mouse to move the camera.

## Documentation

- Chinese version: `README.zh-CN.md`, `debug.zh-CN.md`, and `tutorial/docs/*.zh-CN.md`.
- `tutorial/docs/00_overview.md`: route map and demo index.
- `tutorial/docs/01_programming_basics.md`: loading, stepping, state, control, callbacks, dynamics, model editing, abstract visualization, native UI, and plugin loading.
- `tutorial/docs/02_double_inverted_pendulum_controller.md`: fixed-step two-joint inverted pendulum system with a ros2_control-style controller loop and a GLFW visualization demo.
- `tutorial/docs/03_sensor_suite.md`: built-in and user sensor demo.
- `tutorial/docs/04_rm26_import.md`: RM26 URDF import, path fixes, mesh splitting, and verification.
- `debug.md`: all pitfalls encountered while implementing and verifying the tutorial.
