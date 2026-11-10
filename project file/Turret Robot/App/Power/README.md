# Power 功率/能量反馈与底盘策略（两车公共）

## 数据来源与协议

来源：`docs/RULES_REQUIREMENTS.md`（2026-11-10 读取的规则手册）。

| 字段 | 定义 |
| --- | --- |
| 帧 | 标准 CAN 数据帧，ID `0x31` |
| DLC | 4 |
| 字节 0–1 | 小端有符号 int16，底盘功率，0.1 W/单位 |
| 字节 2–3 | 小端有符号 int16，缓冲能量，0.1 J/单位 |

负值保留，不取绝对值、不截成 0。示例：`F4 01 88 13` → 50 W、500 J；`F6 FF` → −1 J。

## 接收链路

```text
CAN BSP（已校验实例/标准帧/ID/DLC/HAL）
→ 0x31 注册接收器 + 公共 observer（ISR 只保存原始帧和到达时间）
→ app_power_on_can_frame()
→ App_Control 周期 app_power_update()：解析、超时、策略
→ app_motor_submit()：只对左右履带（底盘）施加最终约束
```

- `app_power_on_can_frame()` 只在 ISR 中保存原始字节、总线、DLC 和到达时间，不解析、不控制。
- `app_power_update()` 在任务上下文解析；读取接口不会刷新帧到达时间。
- 总线、超时、硬件核验和规则限制都在 `Config/app_config.c` 的 `power` 段；未确认项各有独立标记。
- 当前没有自动选择“先收到数据”的总线；`source_bus_valid=false` 时不注册接收器。
- 非法 ID/DLC/总线不会更新有效帧的到达时间，不能给旧数据“续命”。

## 接口

| 接口 | 说明 |
| --- | --- |
| `app_power_init(now_ms)` | 清空原始帧/快照/策略 |
| `app_power_on_can_frame(bus,id,data,dlc,arrival_ms)` | ISR 入口，仅保存；由 App_Control 注册 |
| `app_power_update(now_ms)` | 任务上下文解析、超时判断、策略计算 |
| `app_power_get_feedback(out)` | 一致快照：实测功率 W、缓冲能量 J、总线、到达时间、在线/配置/核验/错误位 |
| `app_power_get_policy(out)` | 底盘策略：正常/受限/低能量/不可用、允许功率参考、缩放和边距 |
| `app_power_get_budget(out)` | 旧的归一化代理预算；`available_power_w` 仍为 0，未被实测功率占用 |
| `app_power_limit_scale(total,scale)` | 旧代理缩放入口；不用于底盘 0x31 策略 |

`measured_chassis_power_w` 是裁判反馈真值；`allowed_power_w` 是规则允许参考；`buffer_energy_j` 是裁判反馈能量；`max_total_output_norm` 是旧的归一化代理预算。四者不能混用。

## 超时与恢复

- 上电无有效帧：`frame_valid=false`，来源不可用。
- 持续有效帧：在 `source_timeout_valid` 的窗口内 `source_online=true`。
- 超时：`source_online=false`，增加一次超时计数；旧帧不能被读取动作刷新。
- 非法帧：增加非法计数，不更新有效到达时间。
- 数据恢复和运动恢复分开：App_Control 在底盘功率源失效/降额时清掉旧请求和旧 CAN 队列；恢复后从当前输入重新生成目标，不重放旧动作。
- `hardware_verified=false` 时允许诊断接收，但 `source_ready=false`，不允许底盘运动。

## 底盘策略

规则值来自各车 `Config/app_config.c`：工程车 50 W / 500 J，炮台车 40 W / 400 J；目前规则版本仍有疑点，最终以赛务为准。

- 正常：`chassis_scale=1`。
- 实测功率超上限：`state=LIMITED`，`chassis_scale=limit/measured`，只作为保守比例，不是功率模型，也不承诺瞬时绝不超功率。
- 缓冲能量 ≤ 配置低能量阈值（当前按规则 0 J 下限）：`state=LOW_ENERGY`，`chassis_scale=0`。
- 来源未配置/未核验/离线/超时：`state=UNAVAILABLE`，底盘输出不放行。
- 非底盘机构不使用 0x31 底盘策略；保护层不会因底盘功率源异常把整车所有机构一并判故障。非底盘仍受旧的代理预算和自身保护约束。

## 最终输出约束

`app_motor_submit()` 只对 `TRACK_LEFT/TRACK_RIGHT` 应用底盘策略：

- 速度/电流/电压命令按保守比例处理；`app_motor_output_t.current_limit_norm` 记录最终电流/力矩上限。
- 位置目标**绝不**乘功率比例；受限时直接不输出。
- 该字段是驱动适配层必须消费的最终约束；当前还没有真实电机适配层，因此没有实车电流/力矩限幅。
- 控制周期为 2 ms；策略在当前周期读取反馈、下一输出周期生效。没有前馈/延迟补偿，不能承诺瞬时绝不超功率；余量、饱和和恢复需实车标定。`chassis_scale` 是保守比例，不是瓦数模型。

## 未实现/待确认

- 0x31 实际接 CAN1 还是 CAN2、报文周期和超时阈值。
- 0x31 上板核验（当前 `hardware_verified=false`）。
- 真实电机功率模型、驱动最终电流/力矩闭环参数、响应延迟和余量。
- 低能量阈值/恢复策略的实车标定；当前仅规则 0 J 下限。
- 裁判主控/测速模块接线与 0x31 物理链路。


## 上板步骤（本轮未执行）

1. **只观察接收**：确认 0x31 实际接 CAN1/CAN2 和波特率后，才在 `Config/app_config.c` 设置 `source_bus` 并置 `source_bus_valid=true`；`hardware_verified` 暂时保持 false。电机动力断开，只看 `can_bsp_get_stats()`、`app_power_feedback_t` 的 ID/DLC/总线/到达时间、功率和能量读数。
2. **核对真值**：用裁判/调试工具对照实测功率与缓冲能量，检查正负号、0.1 W/0.1 J 单位和周期；确认帧周期，不把裁判 1 ms 能量结算当成 CAN 周期；设置并验证 `source_timeout_ms`。
3. **允许底盘策略**：确认读数稳定后，才把 `hardware_verified=true`；先在架空/低负载下验证 50 W/40 W 超限降额、低能量停止、位置目标不缩放和 `current_limit_norm` 的驱动侧消费。
4. **断电恢复**：断开/恢复 0x31 来源或底盘供电，确认来源超时后底盘输出撤销、旧队列被清；恢复后只按当前遥控输入生成新目标，不重放旧行进请求；记录响应延迟和余量。
5. **未完成任何一步前**：保持 `hardware_verified=false`，不得把“能解析测试帧”当作实车限功率已完成。

## 验证

- `power_motor_contract_regressions`：真实 Power/Motor/Unit，覆盖字序、单位、正负值、DLC、错误总线、超时、时间回绕、低能量、超功率、位置不缩放、最终 `current_limit_norm`。
- `config_power_engineer_regressions` / `config_power_turret_regressions`：两车 50/500、40/400 配置。
- `production_chain_regressions`：真实 Config/Motor/Power/Protection 未配置路径和急停。
- 以上均为主机测试，未上板；不能据此认定实车限功率完成。

## 快照一致性与隔离

`app_power_update()` 先在局部变量完成解析、超时和策略计算，最后一次短临界区整体发布 feedback/policy/budget/status；读取接口只在短临界区复制快照，不做重试自旋，也不会在写入中途无限等待。解析和策略计算不在长临界区内。旧的归一化预算与 0x31 底盘源独立：非底盘不会因底盘源未配置/离线改变许可；底盘只受 0x31 策略约束。
