# Protection 保护与运行许可模块（两车公共）

## 职责
汇总配置、遥控、设备、功率状态，给出唯一“是否允许输出”的结论与运行状态，管理 active/latched 故障；各机构不得自行解除故障。

## 公开接口与单位
| 接口 | 说明 |
| --- | --- |
| `app_protection_init(now_ms)` | 未使能、无故障；返回 NOT_READY |
| `app_protection_evaluate(now_ms)` | 每周期汇总一次，返回运行状态映射码 |
| `set_arm_request(bool)` / `set_mode_active(bool)` | 只存标志，不自行评估 |
| `app_protection_output_allowed()` | 唯一输出许可 |
| `app_protection_raise(flags)` | 同时置活动位和锁存位，立即撤销输出许可 |
| `app_protection_resolve(flags)` | 监督逻辑确认原因消失后，仅解除指定活动位 |
| `app_protection_clear_latched()` | active 全部清除后才允许人工复位锁存；不自动重新使能 |
| `app_protection_emergency_stop()` / `release_emergency_stop()` / `active()` | 逻辑急停：锁存 MANUAL、撤销使能；物理输入未映射 |
| `app_protection_report()` | 只读报告（const 指针）；时间均为 ms |

## 调用顺序/周期
每周期：`app_motor_clear_requests()` → 各机构 update → `set_arm_request`/`set_mode_active`
→ `app_protection_evaluate(now_ms)` → `app_motor_submit(now_ms)`；必须在 submit 前评估。

## 状态判定顺序与返回码
1. active/latched 非 0 → FAULT，返回 FAULT；2. 配置非 OK → WAIT_CONFIG，返回 UNCONFIGURED；
3. 遥控非 OK → WAIT_INPUT，设备非 OK → WAIT_DEVICE；底盘功率源状态不在这里做整车停用，交给 Motor 的底盘策略。`WAIT_POWER` 枚举保留但当前不作为全局状态。
4. `arm && mode_active` → RUNNING；仅 `arm` → ARMED（均返回 OK）；否则 DISABLED。
5. `APP_FAULT_CAN` 由 App_Control 根据已注册设备的 CAN 健康状态置位；无注册设备时不因占位设备报故障。
`output_allowed` 仅在 ARMED/RUNNING 且无故障时为 true。

## 配置项
无独立配置；只读 `app_config_status()`、`app_remote_status()`、`app_motor_status()`、`app_power_status()`；功率预算由 `app_motor_submit()` 在其后限幅。

## 故障与默认行为
配置未确认 → WAIT_CONFIG；功率预算/来源不可用 → WAIT_POWER；`output_allowed=false`。故障不自动解除，急停释放后仍需 `clear_latched()`。

## 调试步骤
1. 断点看 `app_protection_report()` 的 state、各子状态与故障位；置 `APP_FAULT_MANUAL`
   验证 FAULT 与许可撤销，原因解除后由监督任务调用 `resolve(flags)`，
   确认仍锁存并禁止输出；显式 `clear_latched()` 后重新评估配置/遥控/设备与使能。

所有保护写接口由整车控制/监督任务串行调用；其他任务/ISR 把事件交给监督任务。
不得通过重新初始化模块来绕过活动故障与锁存。

## 负责人
待认领（公共模块，两车同步维护）。

## 未实现项
- 周期超时由 App_Control 在连续 3 个超限周期后 raise；CAN 错误由 App_Control 汇总。功率来源/0x31 仍待接入，失联与功率的自动细分、物理急停映射未完成。
