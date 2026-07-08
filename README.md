# MuJoCo C++ Tutorial Workspace

This workspace contains a step-by-step MuJoCo C++ tutorial built against the bundled MuJoCo SDK at `mujoco/mujoco-3.10.0`.

## Build

```bash
cmake -S . -B build
cmake --build build -j 8
```

The tutorial helper target publishes `TUTORIAL_MODEL_DIR`, `TUTORIAL_RM26_DIR`, and `TUTORIAL_MUJOCO_ROOT` to every demo. Keep those path macros on `mujoco_tutorial_common`; moving them to individual executables breaks shared helpers such as `model_path()` and `rm26_path()`.

## Run All Demos

The active tutorial set is `tutorial_01` through `tutorial_13`.

```bash
for exe in build/bin/tutorial_*; do
    "$exe"
done
```

## Documentation

- Chinese version: `README.zh-CN.md`, `debug.zh-CN.md`, and `tutorial/docs/*.zh-CN.md`.
- `tutorial/docs/00_overview.md`: route map and demo index.
- `tutorial/docs/01_programming_basics.md`: loading, stepping, state, control, callbacks, dynamics, model editing, abstract visualization, native UI, and plugin loading.
- `tutorial/docs/02_double_inverted_pendulum_controller.md`: fixed-step two-joint inverted pendulum system with a ros2_control-style controller loop.
- `tutorial/docs/03_sensor_suite.md`: built-in and user sensor demo.
- `tutorial/docs/04_rm26_import.md`: RM26 URDF import, path fixes, mesh splitting, and verification.
- `debug.md`: all pitfalls encountered while implementing and verifying the tutorial.
