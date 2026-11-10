# Chassis 履带底盘模块（两车公共）

## 职责
把行进/转向请求换算成左右履带角速度请求（差速运动学），只写统一电机请求接口。
不做闭环、不直接操作驱动，也不处理抬升/机械臂。

## 公开接口与单位
| 接口 | 说明 |
| --- | --- |
| `app_chassis_init(now_ms)` | 只清状态；返回 `APP_STATUS_NOT_READY` |
| `app_chassis_update(request, now_ms)` | forward_mps / turn_rad_s（正为左转，待确认）→ 左右 rad/s |
| `app_chassis_get_state(out)` | 拷贝状态快照 |
| `app_chassis_stop(now_ms)` | 清空目标、NOT_READY；不写 0 速度请求 |

## 调用顺序/周期
`app_motor_clear_requests()` 之后调用；每周期一次，位于保护评估之前。无请求或
`request->valid==false` 时只清目标，不写任何电机请求。

## 配置项（`app_config_get()->chassis`）
`track_width_m`、`wheel_radius_m`、`max_linear_speed_mps`、`max_turn_rate_rad_s` 及各自
valid 标记与总开关 `valid`；履带电机用 `APP_MOTOR_ROLE_TRACK_LEFT/RIGHT` 查槽位。

## 换算与判定顺序
1. 无请求 → NOT_READY；2. `chassis.valid==false` → UNCONFIGURED；3. 轮距/轮径/上限缺
valid → UNCONFIGURED；4. NaN/Inf 或超上限 → INVALID_PARAM；5. 履带电机非
motion_usable → NOT_READY；6. `w_left=(forward-turn*W/2)/r`、`w_right=(forward+turn*W/2)/r`，
分别写 `APP_MOTOR_CMD_SPEED_RAD_S` → OK。任何拒绝路径都清空目标、不写请求。

## 故障与默认行为
首轮 `chassis.valid=false`：恒返回 UNCONFIGURED，快照目标为 0，不产生电机请求。
超限请求整体拒绝而非静默限幅；电机口拒绝时两侧目标一并清除。

## 调试步骤
1. 顶起车体用 0.1 m/s 小目标验证方向；2. 核对减速比/轮径后用 `get_state()` 核对 `w=v/r`。

## 负责人
待认领（公共模块，两车同步维护）。

## 未实现项
- 速度闭环、里程计/打滑补偿、越障策略；正方向与转向符号待实车确认。
