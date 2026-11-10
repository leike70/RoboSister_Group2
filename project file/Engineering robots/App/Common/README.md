# App/Common 公共基础类型（两车公共）

## 职责
本目录只放两车共用的最基础定义：状态码、单位约定、时间基准与限幅工具。
不含任何机构逻辑，也不创建 RTOS 对象。两车工程各有一份，修改须同步。

## 公开接口与单位
| 接口 | 单位/说明 |
| --- | --- |
| `app_status_t` | UNCONFIGURED / NOT_READY / OK / OFFLINE / NOT_IMPLEMENTED / INVALID_PARAM / FAULT |
| `app_clampf(v,min,max)` | 浮点限幅到闭区间，纯函数、无副作用 |
| `app_time_now_ms()` | 当前时间 ms，由 `osKernelGetTickCount()` 与 tick 频率换算 |
| `app_time_is_fresh(now,stamp,timeout)` | 新鲜度判断（ms），`APP_TIME_NEVER` 恒不新鲜，处理 32 位回绕 |
| `app_time_elapsed_ms(now,then)` | 时差 ms，无符号减法天然处理回绕 |

单位约定（根 README 第 6 节）：角度 rad、角速度 rad/s、长度 m、时间 ms、归一化量 [-1,1]。
`APP_TIME_NEVER` 用于表达“从未更新”，不得用 0 冒充有效时间戳。

## 调用顺序/周期
`app_time_now_ms()` 只在调度器启动后调用；各模块 update 的时间戳参数由周期任务传入，
模块内部不重复取时间。`app_clampf()` 为纯函数，任意时刻可用。

## 配置项
无。时间基准依赖 FreeRTOS tick 频率，本地工程由 CubeMX 配置，改 tick 不需要改本模块。

## 故障与默认行为
`app_config_status()` 未调用前，各模块状态按各自语义返回 UNCONFIGURED/未实现，
不使用 `app_status_t` 的 0 值冒充“正常”。时间函数不阻塞、不创建对象。

## 调试步骤
1. 在调度器启动后断点读 `app_time_now_ms()`，确认随 tick 递增；
2. 用 `app_time_is_fresh()` 验证超时与回绕（可手工传临界时间戳）；
3. 跨模块调试优先看状态码，不要用返回 0 的数值推断模块正常。

## 负责人
待认领（公共基础，两车同步维护）。

## 未实现项
- 无独立待办；时间基准精度与 tick 换算的实车验证随任务调度一起做。
