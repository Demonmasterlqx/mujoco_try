# MuJoCo Tutorial Debug Notes

This file records the implementation and import pitfalls encountered while building the tutorial. Each item is also reflected in the relevant tutorial document.

## 2026-07-04 - Public Path Macros Must Be Visible To Tutorial Demos

- Symptom: `cmake --build build -j 8` failed while compiling `tutorial/src/common.cpp` because `TUTORIAL_MODEL_DIR` and `TUTORIAL_RM26_DIR` were not declared.
- Cause: the path macros were originally defined only on each demo executable, but `model_path()` and `rm26_path()` live in `mujoco_tutorial_common`. The plugin demo also needs the bundled MuJoCo root path.
- Fix: moved `TUTORIAL_MODEL_DIR`, `TUTORIAL_RM26_DIR`, and `TUTORIAL_MUJOCO_ROOT` to `mujoco_tutorial_common` with public visibility in `tutorial/CMakeLists.txt`.
- Affected docs: the build and programming sections note that path macros are part of the common tutorial target.

## 2026-07-04 - Original RM26 URDF Uses ROS package:// Mesh Paths

- Symptom: MuJoCo's `compile` sample failed to compile the original RM26 URDF and returned `Could not compile model`.
- Cause: the URDF references meshes as `package://rm26_version3_engineering_model/meshes/*.STL`. MuJoCo does not resolve ROS package URLs by default.
- Fix: copied the URDF and meshes into `tutorial/rm26_version3_engineering_model/`, kept the original URDF, and generated MuJoCo-loadable URDF variants with paths rewritten to `../meshes/*.STL`.
- Affected docs: the RM26 import guide explains the original failure and the path rewrite.

## 2026-07-04 - RM26 STL Files Can Exceed MuJoCo's STL Decoder Face Limit

- Symptom: the path-rewritten RM26 URDF failed with `decoder failed for mesh file ... base_link.STL`; the detailed error said the number of STL faces should be between 1 and 200000.
- Cause: `base_link.STL` has 725344 faces and `j4_link.STL` has 245928 faces. MuJoCo's STL decoder rejects a single STL above the 200000-face limit.
- Fix: split only the oversized binary STL files into smaller binary STL parts without dropping triangles. Both MuJoCo import URDF variants now reference split `base_link` and `j4_link` meshes.
- Affected docs: the RM26 import guide explains why split mesh references are needed.

## 2026-07-04 - RM26 end_link Needs Positive Inertial Data

- Symptom: the split RM26 URDF failed with `mass and inertia of moving bodies must be larger than mjMINVAL` for `end_link`.
- Cause: `end_link` is attached through a revolute joint but had no inertial, visual, or collision data in the SolidWorks URDF export.
- Fix: added a small positive inertial block to `end_link` in the MuJoCo import URDF variants.
- Affected docs: the RM26 import guide notes that empty moving links need physically valid inertial data.

## 2026-07-04 - Inverse Dynamics Needs qacc Set After mj_forward

- Symptom: the dynamics demo printed a zero `qfrc_inverse` even though it intended to query inverse dynamics for a nonzero acceleration.
- Cause: `data->qacc[0]` was set before `mj_forward`; `mj_forward` overwrote acceleration as part of forward dynamics.
- Fix: set `data->qacc[0]` immediately before `mj_inverse`.
- Affected docs: the programming basics guide expects a nonzero inverse dynamics output.

## 2026-07-04 - Callback Restoration Should Compare Against Previous Callback

- Symptom: callback demos initially reported restoration by comparing the global callback pointer against `nullptr`.
- Cause: MuJoCo callback globals are process-wide; another application or test could have installed a non-null previous callback.
- Fix: store the previous callback before installing the demo callback, restore it with an RAII guard, and compare against that saved pointer.
- Affected docs: callback sections describe scoped restoration rather than assuming callbacks start as null.

## 2026-07-04 - MuJoCo 3.10 mj_fullM Signature Differs From Older Examples

- Symptom: `tutorial_06_dynamics_queries` failed to compile because `mj_fullM(model, mass_matrix.data(), data->qM)` could not convert the second argument to `const mjData*`.
- Cause: the local MuJoCo 3.10 header declares `mj_fullM(const mjModel* m, const mjData* d, mjtNum* dst)`.
- Fix: changed the call to `mj_fullM(model, data, mass_matrix.data())`.
- Affected docs: the dynamics tutorial uses the local 3.10 signature.

## 2026-07-04 - Prefer mjs_setString For mjSpec String Fields

- Symptom: `tutorial_07_model_editing` segfaulted while assigning the model name through a direct `std::string` write.
- Cause: although `mjString` is mapped to `std::string` in C++ headers, the stable API for variable-length fields is `mjs_setString()`. Direct writes are easier to get wrong across local binary/header details.
- Fix: changed model-name assignment to `mjs_setString(spec->modelname, "programmatic_box")`.
- Affected docs: the model editing tutorial uses setter APIs for variable-length fields.

## 2026-07-04 - mj_saveXMLString Return Convention Is Not Boolean

- Symptom: `tutorial_07_model_editing` compiled its programmatic model but reported failure at `mj_saveXMLString` with no detailed error text.
- Cause: `mj_saveXMLString` returns `0` on success, `-1` on failure, and the required output size if the XML buffer is too small. Treating the result as a boolean inverted the success case.
- Fix: check `save_result != 0` and include the return code in failures. The buffer is still kept generously sized.
- Affected docs: the model editing tutorial calls out the non-boolean return convention.

## 2026-07-04 - Fixed-Step Loops Should Use Integer Step Counts

- Symptom: the double inverted pendulum demo originally ran to `3.001` seconds even though the intended duration was 3 seconds.
- Cause: the first implementation used `while (data->time < 3.0)`. Floating point time and MuJoCo's post-step time update can add one extra step at the boundary.
- Fix: shared stepping now uses `step_count_for_duration()` and integer loop counts; the double pendulum controller runs 3000 iterations at `0.001` seconds.
- Affected docs: the double inverted pendulum guide expects `steps=3000` and `final_time=3`.

## 2026-07-04 - mjuiDef Shortcut Strings Are Strictly Parsed

- Symptom: `tutorial_12_ui_definition` failed at runtime with `mjui_add: invalid shortcut specification`.
- Cause: the demo populated the `other` field for section/check items with casual shortcut strings. MuJoCo's native UI parser expects a specific shortcut format.
- Fix: removed shortcut strings from the headless UI definition demo and left `other` only where it is required for slider ranges.
- Affected docs: the programming basics guide treats the UI demo as a structure-definition example, not a shortcut parser example.

## 2026-07-04 - Rangefinder Direction Is The Site Positive Z Axis

- Symptom: `down_range` in the sensor demo reported `-1`, meaning no geom intersection.
- Cause: rangefinder rays attached to sites use the site's positive Z axis and exclude geoms attached to the same body as the sensor site. Early variants either aimed at empty space or placed the target geom in `worldbody`, the same body as the site.
- Fix: added a `range_target` geom in a child body along the site's positive Z direction and renamed the sensor to `target_range`.
- Affected docs: the sensor suite guide notes that `target_range` should produce a positive hit distance.

## 2026-07-08 - Shared Controller Refactor Needs Qualified Names

- Symptom: after moving the double inverted pendulum controller into shared tutorial files, `tutorial_10_double_inverted_pendulum` failed to compile because `MujocoJointSystem`, `UprightPdController`, and `JointState` were no longer in the executable's local namespace.
- Cause: the first visual demo refactor put reusable controller types in the `mujoco_tutorial` namespace but left the original executable using unqualified names from the previous anonymous-namespace implementation.
- Fix: updated the executable to use `mujoco_tutorial::MujocoJointSystem`, `mujoco_tutorial::UprightPdController`, and `mujoco_tutorial::JointState`.
- Affected docs: the double inverted pendulum guide states that the controller is now shared and namespaced.

## 2026-07-08 - Visual Duration Should Cap Total Fixed Steps

- Symptom: `tutorial_14_visualized_double_pendulum --duration 0.05` initially finished at `final_time=0.051`.
- Cause: the first render loop advanced a full approximate 1/60 second frame chunk each render. With a 0.001 second MuJoCo timestep, that chunk rounds to 17 simulation steps, so the automatic duration boundary could be crossed by one or more timesteps.
- Fix: compute integer `steps_per_frame` and integer `remaining_steps` with `step_count_for_duration()`, then clamp each render frame to the remaining step budget. The same run now reports `simulation_steps=50` and `final_time=0.05`.
- Affected docs: the double inverted pendulum guide explains that rendering frames can group simulation steps but automatic runs still cap the total fixed-step count.

## 2026-07-08 - Run CMake Probes Outside The Source Root

- Symptom: an exploratory `cmake --find-package` check left an untracked top-level `CMakeFiles/` directory in the repository.
- Cause: one-shot CMake find-package mode still writes scratch files relative to the current working directory when run from the source root.
- Fix: removed the generated directory and kept subsequent verification in explicit build directories such as `build/` or `/tmp/...`.
- Affected docs: the overview build section notes that exploratory CMake probes should not be run from the repository root.

## 2026-07-08 - Visualization Changes The Default Text-Mode Assumption

- Symptom: after adding `tutorial_14_visualized_double_pendulum`, the overview still said the tutorial was text-mode by default.
- Cause: that wording was true for the first 13 demos, but the new visualization demo opens a GLFW/OpenGL window unless `--headless-check` is selected.
- Fix: changed the overview to say most demos are text-mode and called out the visual demo's window/headless modes.
- Affected docs: the overview documents the exception.

## 2026-07-08 - Visualization Adds GLFW And OpenGL Build Dependencies

- Symptom: code review found that top-level `find_package(OpenGL REQUIRED)` and `find_package(glfw3 REQUIRED)` would block configuration for users who only need the existing headless demos.
- Cause: the first CMake change made the visualization dependencies global for the whole tutorial subproject.
- Fix: changed the visual demo dependencies to `QUIET`, build `tutorial_14_visualized_double_pendulum` only when GLFW/OpenGL are found, and leave `tutorial_01` through `tutorial_13` buildable without them.
- Affected docs: README and overview now state that CMake builds the windowed demo when it can find `glfw3` and `OpenGL`, otherwise it skips only that demo.

## 2026-07-08 - Duration CLI Parsing Must Be Strict

- Symptom: code review found that `--duration 0.05abc`, `--duration nan`, and `--duration inf` could be accepted by `std::stod`-based parsing.
- Cause: the first parser did not check how many characters `std::stod` consumed and did not reject non-finite values.
- Fix: validate full-string consumption, finite value, and non-negative value before accepting `--duration`.
- Affected docs: the double inverted pendulum guide states that `--duration` must be finite and non-negative.

## 2026-07-08 - Pair GLFW Lifecycle Cleanup

- Symptom: code review found that a successful `glfwInit()` followed by failed `glfwCreateWindow()` entered fallback without an explicit `glfwTerminate()`.
- Cause: the first implementation followed the short MuJoCo sample style and manually destroyed only the normal-path window.
- Fix: added small RAII wrappers so initialized GLFW sessions terminate and created windows are destroyed on all return paths.
- Affected docs: the double inverted pendulum guide records the GLFW lifecycle rule for the visual demo.
