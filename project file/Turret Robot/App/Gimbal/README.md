# App/Gimbal：炮台车云台（Yaw / Pitch）

## 职责
保存 Yaw/Pitch 角度目标，在配置门控下生成本周期统一电机请求。首轮只做
“目标 → `APP_MOTOR_CMD_POSITION_RAD`”，不做速度环、姿态解算、前馈和反陀螺。

## 接口与单位
| 接口 | 说明 |
| --- | --- |
| `app_gimbal_init(now_ms)` | 清状态；云台配置无效 → `UNCONFIGURED` |
| `app_gimbal_set_target(axis, rad)` | 记录单轴目标；越界/非有限 → `INVALID_PARAM` |
| `app_gimbal_get_target(axis, &rad)` | 该轴未设置过 → `NOT_READY` |
| `app_gimbal_update(now_ms)` | 对已设置且配置齐全的轴写 `POSITION_RAD` 请求 |
| `app_gimbal_get_feedback(axis, &fb)` | 无反馈适配：`valid=false`、`NOT_IMPLEMENTED` |
| `app_gimbal_stop(now_ms)` / `get_state(&st)` | 清两轴目标 → `NOT_READY`；快照含 `target_valid` |

轴 `APP_GIMBAL_AXIS_YAW`/`PITCH`；角度 rad、角速度 rad/s。角色：Yaw=GM6020，
Pitch=达妙 6006（不得用 DJI 驱动）。

## 调用顺序
`app_motor_clear_requests()` → `app_vehicle_update()` → `app_gimbal_update()` →
`app_motor_submit()`。本模块不主动清请求，只在条件满足时写本周期请求。

## 配置项（均待确认）
`gimbal.valid`、`gimbal.limits_valid`、Yaw/Pitch 上下限；两台电机的总线、ID、
收发报文 ID、控制模式、方向、减速比、零位、限位、PID。

## 故障行为
`gimbal.valid=false` 时 `init`/`update` 返回 `UNCONFIGURED` 且不写请求；限位未确认时
`set_target` 返回 `INVALID_PARAM`；电机运动配置不齐时不下发。无反馈不填数值，不伪造
限位结论或“到位”判断。

## 调试步骤
1. 先确认达妙 6006 协议适配存在再上电，不得临时套用 DJI 驱动。
2. 单轴低幅度、低速度试转，核对方向与反馈符号；再标零位与机械限位。
3. `limits_valid=true` 后才允许把目标设到全行程。
4. 用 `get_state`/`get_feedback` 观察状态，不把 `OK` 当作已到位。

## 未实现项
速度规划、姿态解算、前馈、反陀螺、达妙驱动、反馈换算。负责人：待认领。
