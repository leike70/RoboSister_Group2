# App/Remote：逻辑输入、原始状态快照、在线判定与三档模式

## 职责

- 保存接收链路发布的归一化逻辑输入（`app_remote_input_t`）与同一帧的原始/协议字段
  （四通道、S1/S2、鼠标、键盘）。
- 每周期用“帧到达时刻”判断在线/失联，生成只读快照。
- 输出三档模式：中档 `APP_MODE_CHASSIS`、上档 `APP_MODE_VEHICLE_UP`、
  下档 `APP_MODE_VEHICLE_DOWN`，另有 UNKNOWN/DISABLED。
- 不直接驱动电机、不实现物理协议；DBUS 解析在 `User_Driver/Remote`，
  串口接收/到达时刻记录在 `Bsp/remote_uart.c`。两车本模块完全一致。

## 接口与单位

| 接口 | 说明 |
| --- | --- |
| `app_remote_init(now_ms)` | 清空输入与两个快照，默认离线 |
| `app_remote_publish(&input)` | 只发布逻辑输入（无原始字段，原始快照保持无效）；供测试/无协议来源使用 |
| `app_remote_publish_frame(&input, &aux)` | 生产入口：同一临界区原子保存逻辑输入 + 原始字段；输入与原始快照来自同一帧 |
| `app_remote_get_aux_snapshot(&aux)` | 读取原始/协议状态快照：`valid`、`timestamp_ms`（到达时刻）、四通道、S1/S2、鼠标、键盘 |
| `app_remote_invalidate(now_ms)` | 立即撤销输入，同时清零原始快照（不残留按下状态） |
| `app_remote_update(now_ms)` | 用 `sample_ms`（到达时刻）检查失联并刷新逻辑快照 |
| `app_remote_get_snapshot(&snap)` | 逻辑快照拷贝（轴、模式、在线、时间戳） |
| `app_remote_status()` | `UNCONFIGURED` / `OFFLINE` / `OK` |

单位：轴为归一化 `[-1,1]`；通道原始值 364..1684；开关 1=上/2=下/3=中；
时间为本地毫秒基准。

## 调用顺序

`remote_uart_service(now)`（解码 → 映射 → 用到达时刻判新鲜度 → `publish_frame`）→
`app_remote_update(now)` → `get_snapshot` / `get_aux_snapshot` → 保护评估 →
中档走 Chassis、上/下档交 Vehicle。各车 `app_vehicle_on_up/on_down` 可自行读取
`app_remote_get_aux_snapshot()`，本模块不绑定任何机构动作。

## 配置项

`Config/app_config.c` 的 `remote` 段：失联门限 `link_timeout_ms`、死区、
通道下标/方向、档位与使能开关选择、各 `*_valid` 门控。任一映射项未确认时
`dbus_to_input` 返回 `UNCONFIGURED`，不会发布假输入。

## 故障与默认行为

- 上电无帧：`OFFLINE`，逻辑/原始快照清零，模式 UNKNOWN，输出许可为假。
- 超过 `link_timeout_ms` 无新帧，或帧到达时刻已超时（任务延迟处理）：
  `OFFLINE`；逻辑与原始快照都作废。
- 非法帧/串口错误：立即撤销输入，原始快照清零，不残留按键/开关状态。
- 物理断开后重新有帧：自动恢复在线；使能只看当前开关，不沿用旧状态。

## 调试步骤

1. 先看 `remote_uart_get_stats()`：有效/非法/过期帧计数、UART 错误、重启。
2. 再看 `app_remote_get_snapshot()`：online、mode、四轴与 enable/reset。
3. 再看 `app_remote_get_aux_snapshot()`：通道原始值、S1/S2、鼠标、键盘是否与操作一致。
4. 最后看保护报告：输入在线只是必要条件，配置/设备/使能仍要满足。

## 负责人

待认领（公共支持：遥控）。

## 未实现项

- 原始字段只提供读取，不映射为动作；通道方向、开关编码、鼠标/键盘语义尚未上板核验。
- 未做跨车遥控或车间通信，本模块只描述本车接收机。
