# Motor 模块：设备注册与统一电机输出

## 职责

- 按 `Config/app_config.c` 注册本车 9 个逻辑电机；配置未确认的电机不注册。
- 维护每台设备的可用性、缺失项、反馈有效性与时间戳。
- 收集各应用模块的每周期请求（staging），由唯一入口 `app_motor_submit()` 结合
  配置门控、保护许可、功率预算和单位换算，生成每台电机唯一最终命令。
- 首轮不实例化任何 `DJI_motor` / 达妙电机对象，不发送 CAN 报文；反馈接口返回
  `NOT_IMPLEMENTED`，不会用 0 冒充有效反馈。

## 文件

| 文件 | 内容 |
| --- | --- |
| `app_motor.h` | 对外接口 |
| `app_motor_internal.h` | 模块内部共享状态（不对外） |
| `app_motor_registry.c` | 注册、反馈快照、设备状态 |
| `app_motor_output.c` | 请求收集、唯一输出提交 |
| `app_unit.h/.c` | 机构单位与转子单位换算（适配层工具） |

## 接口与单位

| 接口 | 说明 |
| --- | --- |
| `app_motor_init(now_ms)` | 按配置注册；不构造硬件对象 |
| `app_motor_update_feedback(now_ms)` | 刷新反馈快照；首轮 `NOT_IMPLEMENTED` |
| `app_motor_feedback_get(slot, out)` | 拷贝反馈快照；无效时 `valid=false` |
| `app_motor_device_get(slot, out)` | 设备状态/缺失项 |
| `app_motor_registered_count()` | 已注册设备数（首轮 0） |
| `app_motor_clear_requests()` | 清空本周期请求 |
| `app_motor_request(slot, mode, value)` | 写请求；`mode` 决定 `value` 单位：电流/电压归一化 `[-1,1]`、速度 `rad/s`、位置 `rad` |
| `app_motor_submit(now_ms)` | **唯一**输出提交入口，每周期一次 |
| `app_motor_output_get(slot, out)` | 最终命令（适配层唯一读取点） |
| `app_motor_disable_all(now_ms)` | 立即失效全部最终命令 |

## 调用顺序

每个控制周期：`app_motor_clear_requests()` → 各模块 `app_motor_request()` →
`app_motor_submit()`。`app_motor_submit` 内部顺序：功率预算 → 保护许可 →
配置/单位换算 → 写唯一最终命令。任何一步不满足，该电机输出 `disabled`。

## 配置门控

- `comms_usable`：总线、电机 ID、报文 ID、控制模式、方向齐全。
- `motion_usable`：另需减速比、零位、限位；速度/位置闭环还需 PID 有效。
- 缺项由 `app_config_motor_missing()` 报告（位标志），缺项电机不参与输出。

## 故障与默认行为

- 上电：无注册设备、无反馈 → `UNCONFIGURED` / `NOT_IMPLEMENTED`。
- 保护模块不允许输出时，`submit` 不使能任何电机，且不保留上一周期命令。
- 功率预算未知时，归一化电流/电压请求按零输出处理；速度/位置请求的功率约束
  由驱动闭环承担，尚未实现，已在 `App/Power/README.md` 登记。

## 调试步骤

1. 断点/观察 `app_motor_registered_count()`，确认未确认配置不会注册设备。
2. 观察 `app_config_motor_missing(slot)`，逐项补齐配置。
3. 硬件适配完成后，确认只有 `app_motor_output_get()` 一处消费最终命令。
4. 用 `app_unit_*.c` 的换算公式核对方向/减速比/零位，禁止在别处重复换算。

## 负责人与状态

- 负责人：待认领（电机与通信）。
- 已实现：注册门控、staging、唯一提交、单位换算门控、旧命令失效。
- 未实现：DJI/达妙硬件适配、反馈解析、CAN 报文提交、功率物理模型。

## 功率门控（本轮）

`app_motor_submit()` 在预算不可用时使所有命令类型失效；仅归一化电流/电压使用 `power_scale`，位置目标绝不被功率缩放。最终硬件适配仍未接入。

## 底盘功率约束

`app_motor_submit()` 只对 `TRACK_LEFT/TRACK_RIGHT` 使用 `app_power_get_policy()`；速度/电流/电压按保守比例，位置目标不缩放、受限时直接不输出。最终命令的 `current_limit_norm` 是驱动适配层必须消费的电流/力矩上限；目前适配层未接，不能声称实车限功率已完成。

## 适配层握手与最终限流

真实驱动适配层必须调用 `app_motor_set_adapter_ready(true)` 后，`app_motor_submit()` 才允许任何 `enabled=true`；未就绪返回 `NOT_IMPLEMENTED`，所有最终命令失效。底盘策略输出 `current_limit_norm`，适配层必须把它限制到最终电流/力矩；当前没有真实适配层，因此生产环境不会输出运动命令。
