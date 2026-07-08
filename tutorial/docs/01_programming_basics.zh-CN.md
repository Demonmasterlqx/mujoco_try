# 编程基础

本文跟随 MuJoCo Programming guide，把每个运行时 API 思路映射到一个小型 C++ demo。

## 1. 加载与步进

Demo：`tutorial_01_load_and_step`

核心流程：

1. 使用 `mj_loadXML` 加载 MJCF。
2. 使用 `mj_makeData` 分配运行时状态和工作区。
3. 使用 `mj_step` 推进一步仿真。
4. 读取 `mjData.qpos` 和 `mjData.qvel`。
5. 用 RAII wrapper 包装 `mj_deleteData` 和 `mj_deleteModel`，自动释放资源。

可复用 wrapper 位于：

- `tutorial/include/mujoco_tutorial/common.hpp`
- `tutorial/src/common.cpp`

运行：

```bash
build/bin/tutorial_01_load_and_step
```

## 2. 状态访问

Demo：`tutorial_02_state_access`

MuJoCo 区分完整的 `mjData` 仿真工作区和紧凑状态向量。这个 demo 保存并恢复一个紧凑状态，包含：

- `mjSTATE_TIME`
- `mjSTATE_QPOS`
- `mjSTATE_QVEL`
- `mjSTATE_CTRL`

展示的 API：

- `mj_stateSize(model, signature)`
- `mj_getState(model, data, buffer, signature)`
- `mj_setState(model, data, buffer, signature)`
- 手动改状态后调用 `mj_forward(model, data)`

运行：

```bash
build/bin/tutorial_02_state_access
```

## 3. 直接控制

Demo：`tutorial_03_control_loop`

如果控制器需要的输入在步进前已经可用，可以直接填充 `mjData.ctrl`，再调用 `mj_step`：

```cpp
data->ctrl[0] = kp * (target - position) - kd * velocity;
mj_step(model, data);
```

该 demo 使用一个带 motor actuator 的单关节机械臂，并打印最终关节状态和传感器表。

运行：

```bash
build/bin/tutorial_03_control_loop
```

## 4. 拆分步进控制

Demo：`tutorial_04_step_split_controller`

`mj_step1` 会先计算与位置、速度相关的派生量；`mj_step2` 再应用控制量并积分。当控制器需要最新运动学、site、传感器或 Jacobian 相关信息时，这种拆分很有用。

流程：

```cpp
mj_step1(model, data);
// 读取最新状态，计算控制量
mj_step2(model, data);
```

运行：

```bash
build/bin/tutorial_04_step_split_controller
```

## 5. 控制回调

Demo：`tutorial_05_callbacks`

MuJoCo 也支持全局控制回调：

```cpp
mjcb_control = control_callback;
```

该 demo 使用一个小型 RAII guard 恢复之前的 callback。这样做很重要，因为 callback 是进程级全局状态，不是每个 model 各自独立的状态。

运行：

```bash
build/bin/tutorial_05_callbacks
```

## 6. 动力学查询

Demo：`tutorial_06_dynamics_queries`

该 demo 展示：

- `mj_fullM(model, data, dense_mass_matrix)`
- `mj_jacSite(model, data, jacp, jacr, site_id)`
- `mj_inverse(model, data)`
- `mj_energyPos(model, data)`
- `mj_energyVel(model, data)`

逆动力学查询会在 `mj_inverse` 前立刻设置 `qacc`；如果在 `mj_forward` 前设置，前向动力学会覆盖它。

运行：

```bash
build/bin/tutorial_06_dynamics_queries
```

## 7. 模型编辑

Demo：`tutorial_07_model_editing`

模型编辑 API 把 XML 加载拆成 parse/edit/compile 三个阶段。这个 demo 不依赖 XML 文件，而是在 C++ 里构造一个小型下落方块：

- `mj_makeSpec`
- `mjs_findBody(spec, "world")`
- `mjs_addGeom`
- `mjs_addBody`
- `mjs_addFreeJoint`
- `mj_compile`
- `mj_saveXMLString`

运行：

```bash
build/bin/tutorial_07_model_editing
```

## 8. 抽象可视化

Demo：`tutorial_08_visualization_scene`

可视化指南把抽象场景构造和 OpenGL 渲染分开。这个 demo 只使用抽象层，所以可以 headless 运行：

- `mjv_defaultCamera`
- `mjv_defaultOption`
- `mjv_defaultScene`
- `mjv_makeScene`
- `mjv_updateScene`
- `mjv_freeScene`

运行：

```bash
build/bin/tutorial_08_visualization_scene
```

## 9. 原生 UI 定义

Demo：`tutorial_12_ui_definition`

Programming guide 里的 UI 部分，用一个 headless 的 `mjUI` 构造 demo 表示。它不渲染界面，但展示 MuJoCo 如何存储原生 UI 的 section 和 item：

- `mjUI`
- `mjuiDef`
- `mjui_themeSpacing`
- `mjui_themeColor`
- `mjui_add`

该 demo 有意避开可选 shortcut 字符串。对带 shortcut 的 item 来说，`mjuiDef.other` 会被严格解析，随手写 `"space"` 这类标签可能让 `mjui_add` 报 `invalid shortcut specification`。在这个 headless 示例中，`other` 只在 slider range 必需的位置使用。

运行：

```bash
build/bin/tutorial_12_ui_definition
```

## 10. 扩展与插件加载

Demo：`tutorial_13_plugin_loading`

Extensions guide 对应的 demo 会从内置 MuJoCo SDK 加载第一方插件库，然后加载 MuJoCo 的插件传感器示例：

- `mj_loadAllPluginLibraries`
- `mjp_getPluginAtSlot`
- `mjData.sensordata` 中的插件传感器数据

运行：

```bash
build/bin/tutorial_13_plugin_loading
```

## Debug 记录

来自 `debug.zh-CN.md` 的相关条目：

- 公共路径宏必须对 common tutorial target 可见。helper library 拥有 `model_path()` 和 `rm26_path()`，plugin demo 还需要 `TUTORIAL_MUJOCO_ROOT`。
- MuJoCo 3.10 声明的是 `mj_fullM(const mjModel*, const mjData*, mjtNum*)`；要以本地 header 为准，不要照搬旧示例。
- 对 `mjSpec` 的可变长度字段，优先使用 `mjs_setString` 这类 setter API。
- `mj_saveXMLString` 不是 boolean：`0` 表示成功，`-1` 表示失败，正数可能表示需要的输出 buffer 大小。
- 固定时长步进应基于 `model->opt.timestep` 计算整数步数。
- callback demo 应和之前保存的 callback 指针比较，不要和 `nullptr` 比较，因为 callback 是全局进程状态。
- 原生 `mjuiDef` shortcut 字符串会被严格解析；headless 结构 demo 中除非使用 MuJoCo 期望的 shortcut 语法，否则不要填写它。
