# Lift 前后抬升模块（两车公共）

## 职责
把前/后机构目标角度（rad）转成两台 6020 的位置请求，只写统一电机请求接口。
首轮不做自动越障、不做前后同步闭环，也不做机械自锁/制动。

## 公开接口与单位
| 接口 | 说明 |
| --- | --- |
| `app_lift_init(now_ms)` | 只清状态；返回 `APP_STATUS_NOT_READY` |
| `app_lift_update(request, now_ms)` | front/rear_target_rad（rad）→ 两条 `APP_MOTOR_CMD_POSITION_RAD` |
| `app_lift_get_state(out)` | 拷贝状态快照（无反馈时 front/rear_valid=false） |
| `app_lift_stop(now_ms)` | 清目标、NOT_READY；不做位置归零 |

## 调用顺序/周期
`app_motor_clear_requests()` 之后、保护评估之前，每周期一次。无请求或
`request->valid==false` → NOT_READY，不写任何请求。

## 配置项（`app_config_get()->lift`）
`travel_min_rad`/`travel_max_rad` 行程、`sync_tolerance_rad` 同步容差（当前未使用）、
`travel_valid`/`sync_valid`/`valid`；槽位用 `APP_MOTOR_ROLE_LIFT_FRONT/REAR`。

## 判定顺序
1. 无请求 → NOT_READY；2. `lift.valid==false` → UNCONFIGURED；3. NaN/Inf → INVALID_PARAM；
4. 行程未配置 → UNCONFIGURED；5. 目标越界 → INVALID_PARAM（不写）；6. 抬升电机非
motion_usable → NOT_READY；7. 写前/后位置请求 → OK。拒绝路径一律清空快照。

## 故障与默认行为
首轮 `lift.valid=false`：恒返回 UNCONFIGURED，不产生位置请求。无反馈时
`sync_error_rad` 保持 0、`front_valid/rear_valid=false`，0 不代表有效测量。
`stop()` 只清目标与状态；机械自锁/制动/支撑策略是明确待办。

## 调试步骤
1. 先单独低速验证前/后方向与零位、行程上下限；2. 核对减速比与 6020 位置换算后打开 travel_valid，空载小幅验证。

## 负责人
待认领（公共模块，两车同步维护）。

## 未实现项
- 前后同步闭环、越障动作、自锁/制动、零位标定与限位开关；行程与同步参数待测。
