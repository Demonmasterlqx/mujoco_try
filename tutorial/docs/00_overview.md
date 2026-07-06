# MuJoCo C++ Tutorial Overview

Official reference used for this tutorial:

- MuJoCo Programming guide: https://mujoco.readthedocs.io/en/stable/programming/index.html
- Simulation: https://mujoco.readthedocs.io/en/stable/programming/simulation.html
- Visualization: https://mujoco.readthedocs.io/en/stable/programming/visualization.html
- Model Editing: https://mujoco.readthedocs.io/en/stable/programming/modeledit.html
- Code samples: https://mujoco.readthedocs.io/en/stable/programming/samples.html
- Extensions and plugins: https://mujoco.readthedocs.io/en/stable/programming/extension.html
- XML sensor reference: https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor

The code is intentionally small and text-mode by default. Every feature below has a runnable demo in `build/bin/`; the current tutorial set is `tutorial_01` through `tutorial_13`.

## Demo Map

| Demo | Source | Model | Purpose |
| --- | --- | --- | --- |
| `tutorial_01_load_and_step` | `tutorial/src/01_load_and_step.cpp` | `free_box.xml` | `mj_loadXML`, `mj_makeData`, `mj_step`, `qpos/qvel` |
| `tutorial_02_state_access` | `tutorial/src/02_state_access.cpp` | `hinge_arm.xml` | `mj_stateSize`, `mj_getState`, `mj_setState`, sensors |
| `tutorial_03_control_loop` | `tutorial/src/03_control_loop.cpp` | `hinge_arm.xml` | direct `mjData.ctrl` control before `mj_step` |
| `tutorial_04_step_split_controller` | `tutorial/src/04_step_split_controller.cpp` | `hinge_arm.xml` | `mj_step1`/`mj_step2` split control |
| `tutorial_05_callbacks` | `tutorial/src/05_callbacks.cpp` | `hinge_arm.xml` | `mjcb_control` callback install/restore |
| `tutorial_06_dynamics_queries` | `tutorial/src/06_dynamics_queries.cpp` | `hinge_arm.xml` | mass matrix, Jacobian, inverse dynamics, energy |
| `tutorial_07_model_editing` | `tutorial/src/07_model_editing.cpp` | generated in C++ | `mj_makeSpec`, `mjs_add*`, `mj_compile`, `mj_saveXMLString` |
| `tutorial_08_visualization_scene` | `tutorial/src/08_visualization_scene.cpp` | `hinge_arm.xml` | `mjvScene` abstract visualization without OpenGL |
| `tutorial_09_sensor_suite` | `tutorial/src/09_sensor_suite.cpp` | `sensor_suite.xml` | built-in sensors plus `mjcb_sensor` user sensor |
| `tutorial_10_double_inverted_pendulum` | `tutorial/src/10_double_inverted_pendulum.cpp` | `double_inverted_pendulum.xml` | fixed-step read-update-write controller loop |
| `tutorial_11_rm26_import_check` | `tutorial/src/11_rm26_import_check.cpp` | RM26 split URDF | verifies imported RM26 model loads in MuJoCo |
| `tutorial_12_ui_definition` | `tutorial/src/12_ui_definition.cpp` | none | native `mjUI` section and item definition |
| `tutorial_13_plugin_loading` | `tutorial/src/13_plugin_loading.cpp` | MuJoCo plugin sample | loads first-party plugin libraries and a plugin sensor model |

## Build And Verification

```bash
cmake -S . -B build
cmake --build build -j 8
for exe in build/bin/tutorial_*; do "$exe"; done
```

Build-time path macros are intentionally attached to the common tutorial target with public visibility:

- `TUTORIAL_MODEL_DIR` for XML models used by multiple demos.
- `TUTORIAL_RM26_DIR` for the copied RM26 import tree.
- `TUTORIAL_MUJOCO_ROOT` for first-party plugin samples under the bundled SDK.

Do not move these macros down to individual executables unless the common helper API changes too.

The latest local verification completed all 13 tutorial demos with exit code 0. Key output checks:

- `tutorial_10_double_inverted_pendulum` reports `steps=3000`, `fixed_period=0.001`, and `final_time=3`.
- `tutorial_09_sensor_suite` reports `target_range` with a positive hit distance, currently `0.560000`.
- `tutorial_11_rm26_import_check` reports:

```text
model=rm26_version3_engineering_model nq=13 nv=13 nu=0 nsensor=0 timestep=0.002
nbody=14 njnt=13 ngeom=18 nmesh=18
```

The direct MuJoCo RM26 URDF and the explicit split URDF also compile with MuJoCo's bundled `compile` sample:

```bash
mujoco/mujoco-3.10.0/bin/compile tutorial/rm26_version3_engineering_model/urdf/rm26_version3_engineering_model_mujoco.urdf
mujoco/mujoco-3.10.0/bin/compile tutorial/rm26_version3_engineering_model/urdf/rm26_version3_engineering_model_mujoco_split.urdf
```

## Debug Notes

All pitfalls are tracked in `debug.md`. Each topic document repeats the relevant items near the feature that triggers them.
