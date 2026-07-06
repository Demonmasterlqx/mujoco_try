# Sensor Suite

Demo: `tutorial_09_sensor_suite`

Model: `tutorial/models/sensor_suite.xml`

The sensor demo covers both built-in sensors and a user sensor backed by `mjcb_sensor`.

## Sensors In The MJCF

The model defines these sensor categories:

- contact: `touch`
- IMU-like: `accelerometer`, `velocimeter`, `gyro`, `magnetometer`
- wrench: `force`, `torque`
- ray: `rangefinder`
- joint and actuator: `jointpos`, `jointvel`, `actuatorfrc`, `jointactuatorfrc`
- frame: `framepos`, `framequat`, `framelinvel`, `frameangvel`
- subtree: `subtreecom`, `subtreelinvel`, `subtreeangmom`
- time: `clock`
- custom: `user`

The C++ demo iterates over:

- `mjModel.nsensor`
- `mjModel.sensor_adr`
- `mjModel.sensor_dim`
- `mjData.sensordata`

This is the robust way to print sensor output because not every sensor has the same dimension.

## User Sensor Callback

The custom sensor is declared in XML:

```xml
<user name="user_link_state" objtype="body" objname="link1" dim="2" needstage="vel"/>
```

The callback fills the allocated sensor slice when MuJoCo reaches the velocity stage:

```cpp
void user_sensor_callback(const mjModel* model, mjData* data, int stage) {
    if (stage != mjSTAGE_VEL) {
        return;
    }
    // fill data->sensordata[sensor_adr + i]
}
```

The demo uses an RAII guard to restore the previous `mjcb_sensor` callback because callback pointers are global.

## Run

```bash
build/bin/tutorial_09_sensor_suite
```

Expected verification signal:

- `nsensor=21`
- `nsensordata=49`
- a positive `target_range` row, currently `0.560000`
- a row for `user_link_state`
- `sensor_callback_restored=true`

## Debug Notes

Relevant entries from `debug.md`:

- Shared model paths must be compile definitions on `mujoco_tutorial_common`, otherwise sensor demos cannot locate `sensor_suite.xml`.
- User callbacks should be installed with scoped restoration to avoid leaking global callback state into later demos.
- Rangefinder sensors measure along the site positive Z axis and exclude geoms attached to the same body as the sensor site.
