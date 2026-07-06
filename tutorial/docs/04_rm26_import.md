# RM26 Model Import

Source model:

```text
~/code/RM/RMController/core/models/rm26_version3_engineering_model
```

Imported copy:

```text
tutorial/rm26_version3_engineering_model
```

Verification demo:

```bash
build/bin/tutorial_11_rm26_import_check
```

## Copied Files

The import copied the URDF and mesh payload needed by MuJoCo:

```text
urdf/rm26_version3_engineering_model.urdf
meshes/*.STL
```

The ROS package files, launch files, RViz config, and export log are not needed by MuJoCo runtime loading.

## Import Variants

Three URDF variants are kept:

- `rm26_version3_engineering_model.urdf`: original SolidWorks/ROS URDF.
- `rm26_version3_engineering_model_mujoco.urdf`: direct MuJoCo import URDF with `package://rm26_version3_engineering_model/meshes/` rewritten to `../meshes/`, oversized mesh references split, and valid inertial data for `end_link`.
- `rm26_version3_engineering_model_mujoco_split.urdf`: equivalent verified import entry kept as the explicit split-mesh variant.

The demo loads the split version.

## Verification Result

Latest local run:

```text
model=rm26_version3_engineering_model nq=13 nv=13 nu=0 nsensor=0 timestep=0.002
nbody=14 njnt=13 ngeom=18 nmesh=18
first_body=image_pitch_link
time=0
```

This proves the model imports into the local MuJoCo runtime and can allocate `mjData` and run `mj_forward`.

MuJoCo's `compile` sample succeeds on both MuJoCo import URDFs when run without an output file:

```bash
mujoco/mujoco-3.10.0/bin/compile tutorial/rm26_version3_engineering_model/urdf/rm26_version3_engineering_model_mujoco.urdf
mujoco/mujoco-3.10.0/bin/compile tutorial/rm26_version3_engineering_model/urdf/rm26_version3_engineering_model_mujoco_split.urdf
```

The original ROS URDF still fails as-is in the same tool. ROS `package://` paths are the first blocker, and the MuJoCo import URDFs also apply the mesh split and `end_link` inertial fixes.

Reproduce the expected original-URDF failure with:

```bash
mujoco/mujoco-3.10.0/bin/compile tutorial/rm26_version3_engineering_model/urdf/rm26_version3_engineering_model.urdf
```

## Debug Notes

Relevant entries from `debug.md`:

- Original URDF uses ROS `package://` mesh paths, which MuJoCo does not resolve by default.
- `base_link.STL` and `j4_link.STL` exceed MuJoCo's STL decoder limit of 200000 faces per STL file. They were split into multiple binary STL files without dropping triangles.
- `end_link` is a moving body attached by a revolute joint but had no inertial data. The MuJoCo import URDFs add a small positive inertial block.

## Remaining Modeling Notes

The imported RM26 model is passive:

- `nu=0`, because the URDF has no MuJoCo actuators.
- Several source joint limits have `effort="0"` and `velocity="0"`.
- Dense CAD meshes are still used for visual/collision geometry. The split fix makes loading succeed, but a production simulation should add simplified collision geometry for speed and contact quality.
