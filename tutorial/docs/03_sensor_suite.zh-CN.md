# 传感器套件

Demo：`tutorial_09_sensor_suite`

模型：`tutorial/models/sensor_suite.xml`

传感器 demo 覆盖内置传感器，以及一个由 `mjcb_sensor` 支持的用户自定义传感器。

## MJCF 中的传感器

模型定义了这些传感器类别：

- contact：`touch`
- IMU-like：`accelerometer`、`velocimeter`、`gyro`、`magnetometer`
- wrench：`force`、`torque`
- ray：`rangefinder`
- joint 和 actuator：`jointpos`、`jointvel`、`actuatorfrc`、`jointactuatorfrc`
- frame：`framepos`、`framequat`、`framelinvel`、`frameangvel`
- subtree：`subtreecom`、`subtreelinvel`、`subtreeangmom`
- time：`clock`
- custom：`user`

C++ demo 会遍历：

- `mjModel.nsensor`
- `mjModel.sensor_adr`
- `mjModel.sensor_dim`
- `mjData.sensordata`

这是打印传感器输出的稳妥方式，因为不同传感器的维度并不相同。

## 用户传感器回调

自定义传感器在 XML 中这样声明：

```xml
<user name="user_link_state" objtype="body" objname="link1" dim="2" needstage="vel"/>
```

当 MuJoCo 到达 velocity stage 时，callback 填写分配给该传感器的 slice：

```cpp
void user_sensor_callback(const mjModel* model, mjData* data, int stage) {
    if (stage != mjSTAGE_VEL) {
        return;
    }
    // 填充 data->sensordata[sensor_adr + i]
}
```

该 demo 使用 RAII guard 恢复之前的 `mjcb_sensor` callback，因为 callback 指针是全局状态。

## 运行

```bash
build/bin/tutorial_09_sensor_suite
```

预期验证信号：

- `nsensor=21`
- `nsensordata=49`
- `target_range` 行的值为正，当前是 `0.560000`
- 有一行 `user_link_state`
- `sensor_callback_restored=true`

## Debug 记录

来自 `debug.zh-CN.md` 的相关条目：

- 共享模型路径必须作为 compile definition 挂在 `mujoco_tutorial_common` 上，否则传感器 demo 找不到 `sensor_suite.xml`。
- 用户 callback 应使用 scoped restoration 安装，避免把全局 callback 状态泄漏到后续 demo。
- rangefinder 传感器沿 site 的正 Z 轴测量，并且会排除与传感器 site 绑定在同一 body 上的 geom。
