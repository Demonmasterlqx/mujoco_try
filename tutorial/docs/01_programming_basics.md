# Programming Basics

This document follows the MuJoCo Programming guide and maps each runtime API idea to a small C++ demo.

## 1. Loading And Stepping

Demo: `tutorial_01_load_and_step`

Core flow:

1. Load MJCF with `mj_loadXML`.
2. Allocate runtime state/workspace with `mj_makeData`.
3. Step the simulation with `mj_step`.
4. Read `mjData.qpos` and `mjData.qvel`.
5. Free with RAII wrappers around `mj_deleteData` and `mj_deleteModel`.

The reusable wrappers live in:

- `tutorial/include/mujoco_tutorial/common.hpp`
- `tutorial/src/common.cpp`

Run:

```bash
build/bin/tutorial_01_load_and_step
```

## 2. State Access

Demo: `tutorial_02_state_access`

MuJoCo distinguishes the full `mjData` simulation workspace from compact state vectors. This demo saves and restores a compact state containing:

- `mjSTATE_TIME`
- `mjSTATE_QPOS`
- `mjSTATE_QVEL`
- `mjSTATE_CTRL`

APIs shown:

- `mj_stateSize(model, signature)`
- `mj_getState(model, data, buffer, signature)`
- `mj_setState(model, data, buffer, signature)`
- `mj_forward(model, data)` after manual state edits

Run:

```bash
build/bin/tutorial_02_state_access
```

## 3. Direct Control

Demo: `tutorial_03_control_loop`

For simple controllers whose inputs are already available before stepping, fill `mjData.ctrl` and call `mj_step`:

```cpp
data->ctrl[0] = kp * (target - position) - kd * velocity;
mj_step(model, data);
```

The demo uses a one-joint arm with a motor actuator and prints the final joint state plus sensor table.

Run:

```bash
build/bin/tutorial_03_control_loop
```

## 4. Split Step Control

Demo: `tutorial_04_step_split_controller`

`mj_step1` computes position/velocity-dependent derived quantities before control. `mj_step2` applies controls and integrates. This is useful when a controller needs fresh kinematics, sites, sensors, or Jacobian-related quantities.

Flow:

```cpp
mj_step1(model, data);
// read fresh state, compute control
mj_step2(model, data);
```

Run:

```bash
build/bin/tutorial_04_step_split_controller
```

## 5. Control Callback

Demo: `tutorial_05_callbacks`

MuJoCo also supports a global control callback:

```cpp
mjcb_control = control_callback;
```

The demo uses a small RAII guard to restore the previous callback. This matters because callbacks are global process state, not per-model state.

Run:

```bash
build/bin/tutorial_05_callbacks
```

## 6. Dynamics Queries

Demo: `tutorial_06_dynamics_queries`

The demo shows:

- `mj_fullM(model, data, dense_mass_matrix)`
- `mj_jacSite(model, data, jacp, jacr, site_id)`
- `mj_inverse(model, data)`
- `mj_energyPos(model, data)`
- `mj_energyVel(model, data)`

The inverse dynamics query sets `qacc` immediately before `mj_inverse`; setting it before `mj_forward` would be overwritten by forward dynamics.

Run:

```bash
build/bin/tutorial_06_dynamics_queries
```

## 7. Model Editing

Demo: `tutorial_07_model_editing`

The model editing API breaks XML loading into parse/edit/compile steps. This demo builds a small falling box without an XML file:

- `mj_makeSpec`
- `mjs_findBody(spec, "world")`
- `mjs_addGeom`
- `mjs_addBody`
- `mjs_addFreeJoint`
- `mj_compile`
- `mj_saveXMLString`

Run:

```bash
build/bin/tutorial_07_model_editing
```

## 8. Abstract Visualization

Demo: `tutorial_08_visualization_scene`

The visualization guide separates abstract scene construction from OpenGL rendering. This demo only uses the abstract layer, so it runs headless:

- `mjv_defaultCamera`
- `mjv_defaultOption`
- `mjv_defaultScene`
- `mjv_makeScene`
- `mjv_updateScene`
- `mjv_freeScene`

Run:

```bash
build/bin/tutorial_08_visualization_scene
```

## 9. Native UI Definitions

Demo: `tutorial_12_ui_definition`

The Programming guide's UI section is represented by a headless `mjUI` construction demo. It does not render, but it shows how MuJoCo stores native UI sections and items:

- `mjUI`
- `mjuiDef`
- `mjui_themeSpacing`
- `mjui_themeColor`
- `mjui_add`

The demo deliberately avoids optional shortcut strings. `mjuiDef.other` is strictly parsed for shortcut-bearing items, and casual labels such as `"space"` can make `mjui_add` fail with `invalid shortcut specification`. In this headless example, `other` is used only for the slider range where the UI API requires it.

Run:

```bash
build/bin/tutorial_12_ui_definition
```

## 10. Extension And Plugin Loading

Demo: `tutorial_13_plugin_loading`

The Extensions guide is represented by loading the first-party plugin libraries from the bundled MuJoCo SDK and then loading MuJoCo's plugin sensor sample:

- `mj_loadAllPluginLibraries`
- `mjp_getPluginAtSlot`
- plugin-backed sensor data in `mjData.sensordata`

Run:

```bash
build/bin/tutorial_13_plugin_loading
```

## Debug Notes

Relevant entries from `debug.md`:

- Public path macros must be visible through the common tutorial target. The helper library owns `model_path()` and `rm26_path()`, and the plugin demo needs `TUTORIAL_MUJOCO_ROOT`.
- MuJoCo 3.10 declares `mj_fullM(const mjModel*, const mjData*, mjtNum*)`; use the local header, not older snippets.
- For `mjSpec` variable-length fields, prefer setter APIs such as `mjs_setString`.
- `mj_saveXMLString` is not boolean: `0` is success, `-1` is failure, and a positive value can mean the required output size.
- Fixed-duration stepping should use integer step counts derived from `model->opt.timestep`.
- Callback demos compare against the previous callback pointer, not `nullptr`, because callbacks are global process state.
- Native `mjuiDef` shortcut strings are strictly parsed; omit them in the headless structure demo unless using MuJoCo's expected shortcut syntax.
