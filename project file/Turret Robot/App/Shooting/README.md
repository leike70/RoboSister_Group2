# App/Shooting：炮台车摩擦轮与拨弹

## 职责
保存摩擦轮目标转速与拨弹请求，在配置门控下生成本周期统一电机请求；维护卡弹状态。
首轮不实现卡弹检测、定量拨弹和发射成功判定。

## 接口与单位
| 接口 | 说明 |
| --- | --- |
| `app_shooting_init(now_ms)` | 清状态；发射配置无效 → `UNCONFIGURED` |
| `app_shooting_update(&req, now_ms)` | 请求无效清目标；有效则写摩擦轮/拨弹请求 |
| `app_shooting_get_state(&st)` | 快照：摩擦轮目标、`firing`、`jam` |
| `app_shooting_stop(now_ms)` | 清摩擦轮/拨弹目标 → `NOT_READY` |

请求字段单位：`friction_left_rad_s`、`friction_right_rad_s`，rad/s（正负方向待实测）。
角色：`FRICTION_LEFT`/`FRICTION_RIGHT` = 3508，`TRIGGER` = 2006。

## 发射使能条件（当前实现）
1. 遥控快照有效、保护 `output_allowed=true`（由 `App/Vehicle/` 保证）。
2. `config.shooting.valid=true`，两只摩擦轮电机运动配置齐全。
3. 摩擦轮目标成对有效、有限，绝对值不超过 `friction_max_rad_s`。
4. 摩擦轮请求写入成功后，才允许记录发射请求；拨弹转速未标定，不写拨弹占位请求。

## 故障与默认行为
- 请求无效/失联/保护未放行：`update` 或 `stop` 清空摩擦轮/拨弹目标，返回 `NOT_READY`。
- 配置无效：返回 `UNCONFIGURED`，不写任何电机请求。
- 摩擦轮目标非法或超 `friction_max_rad_s`：返回 `INVALID_PARAM`，本周期两轮都不写。
- 无卡弹检测，`jam` 恒 `UNKNOWN`；不把“无请求”当作已停转或已排除卡弹。


## 卡弹状态机（未实现）
`jam` 恒为 `APP_SHOOT_JAM_UNKNOWN`；`jam_reset` 只记录“收到复位请求”，不清除、不升级、
不声称已恢复。待实现：堵转/转速偏差判据、确认与恢复流程、拨弹定量。

## 调试步骤
1. 空载单转摩擦轮：核对方向、反馈符号、转速闭环与两只轮一致性。
2. 标定 `friction_max_rad_s`，确认 `friction_valid=true`。
3. 单独调试拨弹：小步低速，确认能量取、无卡弹。
4. 联调发射条件；`output_allowed=false` 时不得有任何输出。

## 未实现项与待确认
拨弹转速来源未确认：`trigger_rate_hz` 与每圈弹量换算、2006 减速比均未标定；
`update` 在 `fire==true` 时只记录“收到发射请求但无法执行”，**不写任何拨弹占位请求**，
不能当作发射速度或发射成功。卡弹检测/恢复、发射计数、摩擦轮闭环整定均未实现。
负责人：待认领。

## 调用顺序
`app_shooting_init` → 每周期本车接入层生成请求并调用 `app_shooting_update(&req, now)` →
最后统一由 `app_motor_submit()` 提交；无请求/失联时调用 `app_shooting_stop`。
本模块只写 staging 请求，不直接提交输出、不创建 RTOS 对象。

