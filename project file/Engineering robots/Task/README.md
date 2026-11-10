# Task 模块：整车启动与周期调度

## 职责

- 提供唯一的整车应用入口 `app_control_task()`；`App_Control` 已在 `.ioc` 的 FreeRTOS Tasks 登记，由 `Core/Src/freertos.c` 创建为单独线程。
- 提供独立的 `imu_task()`：IMU 初始化/校准/2 ms 采样与 EKF 解算，避免传感器等待阻塞 App_Control。
- 首次运行时按固定顺序初始化各模块；之后按 `APP_CONTROL_PERIOD_MS = 2 ms`
  执行一次整车控制流水线。
- 只做调度与顺序约束，不实现驱动、控制算法或运动学。
- 已有 CAN1/CAN2 发送任务、DJI 电机服务任务保持不变，本模块不创建也不修改它们；
  当前共五个 RTOS 任务。

## 公开接口

| 接口 | 说明 |
| --- | --- |
| `app_control_task(void *argument)` | 整车应用线程入口；禁止在中断中调用 |
| `App_Control_attributes` | `freertos.c` 中对应 `.ioc` 的任务属性：512 words、AboveNormal |
| `g_app_control_task_handle` | 应用线程句柄（调试用） |
| `app_task_get_report(app_task_report_t *)` | 读取周期计数、超时次数、模块状态、遥控原始快照状态、姿态/IMU 状态、校准进度、加热占空比与看门狗状态 |
| `imu_task(void *argument)` | IMU 采集/温控/解算线程入口；与 App_Control 相互独立 |
| `IMU_Acquire_attributes` / `imu_task_get_report()` | `.ioc` 中的 IMU 任务属性（2048 words、Normal）与报告（状态机、故障位、传感器结果、温度/目标/加热占空比、样本/错误、校准进度、零偏、最近原始样本） |
| `APP_CONTROL_PERIOD_MS` | 应用控制周期，ms；未做实时性验证 |

## 调用顺序（每周期）

1. `remote_uart_service(now)`：消费 USART3/DMA 收到的 DBUS 帧（解析→映射→用帧到达
   时刻判新鲜度→发布）；随后 `app_remote_update` / `app_remote_get_snapshot` /
   `app_remote_get_aux_snapshot` / `app_motor_update_feedback` / `app_attitude_update` /
   `app_attitude_get_snapshot` / `imu_task_get_report` / `app_power_update` 取得输入与反馈快照。
   `imu_heater_supervise(now)` 在这里执行加热看门狗：IMU 任务停止刷新 300 ms 后强制 CCR=0。
   报告中的 `remote_aux_valid`/`remote_aux_ms`、`attitude_valid`/`attitude_ms`、
   `heater_duty`/`heater_stale` 反映对应状态。
2. 逻辑复位请求上升沿 → `app_protection_clear_latched`（复位请求来自 DBUS 右开关下档；
   保护写接口由控制任务串行调用）。
3. `app_protection_set_arm_request` / `app_protection_set_mode_active` /
   `app_protection_evaluate`：健康与模式判断。
4. `app_motor_clear_requests`：清除上一周期请求。
5. 按模式生成底盘/抬升请求：中档生成公共底盘请求；上档/下档不经过本文件，
   由 `app_vehicle_update` 分派到本车 `app_vehicle_on_up()` / `app_vehicle_on_down()`
   （当前均返回 `NOT_IMPLEMENTED`，不产生请求）。抬升映射待确认，当前不生成。
6. `app_chassis_update` / `app_lift_update` / `app_vehicle_update`。
7. `app_motor_submit`：唯一输出提交入口。
8. 输出许可下降沿：`app_motor_disable_all` + `app_lift_stop` + `app_vehicle_stop`。
9. 更新周期计数、超时统计与报告。

遥控接收由 DMA/IDLE 回调保存原始帧，解析与发布在 `App_Control` 任务上下文完成，
不新增遥控任务；中断内不做映射/保护判断。

## IMU_Acquire 任务

`imu_task` 独立于 App_Control，启动后按非阻塞状态机循环：

```text
INIT → TEMP_PREP → CALIBRATING → RUNNING
  ↑         ↑            │
  └── FAULT（500 ms 后重试）
```

1. `INIT`：`imu_heater_stop()` / 温控积分清零 / `bmi088_initialize()`；SPI 读 ID、软复位、
   配置并回读；等待用 `osDelay` 让出 CPU。
2. `TEMP_PREP`：2 ms 读 `bmi088_sample()`，通过 `imu_temperature_update()`/`imu_heater` 闭环加热；
   只有有效温度才输出非零占空比。目标 ±1 °C 持续 3 s 才进入校准；预热超 120 s 置故障。
3. `CALIBRATING`：静止 500 样本求零偏；移动重新累计；温度偏离目标 >2 °C 回到 TEMP_PREP；
   60 s 未完成则置校准超时。
4. `RUNNING`：`imu_fusion_push()` 正常解算并经 `app_attitude_ingest()` 发布；温度偏离 >3 °C
   回 TEMP_PREP，EKF/传感器错误进入 FAULT。
5. `FAULT`：立即关热、清温控积分、撤销姿态；500 ms 后回到 INIT，禁止无限等待。
6. `App_Control` 每周期调用 `imu_heater_supervise()`；IMU 任务停止刷新时 300 ms 内强制 CCR=0。

IMU 任务不参与三档分派、不写电机请求；其初始化和热准备不会阻塞 App_Control。
线程属性：`IMU_Acquire` 优先级 Normal、栈 8192 B、目标周期 2 ms；`App_Control`
优先级 AboveNormal、栈 2048 B、周期 2 ms。均为静态配置，实际栈高水位和 CPU 占用待上板测量。

## 配置

- 周期长度在本文件定义；硬件映射与参数来自 `Config/app_config.c`，IMU/温控参数来自
  `Config/imu_config.c` 的 `temperature` 段（目标、PWM 上限、PID、就绪/保护阈值）。
- 初始化不等待硬件：未接入的模块返回 `UNCONFIGURED` / `NOT_IMPLEMENTED`，
  线程继续运行并维护状态。

## 故障与默认行为

- 上电后配置未确认、无设备反馈 → 保护状态 `WAIT_CONFIG`，输出永远不允许；
  默认无动作输出。
- 接收机无帧/超时 → `App/Remote` 报 `OFFLINE`，保护进入 `WAIT_INPUT`；
  非法帧或 UART 错误立即撤销输入，不沿用旧快照。
- IMU 初始化/温度准备/校准期间姿态无效；温度过期、过温、升温异常、传感器错误或 EKF 故障
  会立即关热并撤销姿态，500 ms 后重新走 INIT→TEMP_PREP→CALIBRATING；不阻塞整车调度。
- App_Control 独立执行加热看门狗，IMU 任务停更时旧占空比不会一直保持。
- 周期超时只统计 `overrun_count`，不直接置故障，避免单次抖动锁死整车。
- 失联/模式切换/故障时，请求每周期重清、旧命令显式失效。

## 调试步骤

1. 在调试器中查看 `app_task_get_report()`：确认 `cycle_count` 递增。
2. 先看 `remote_uart_get_stats()`：有效帧应随接收机周期递增，非法帧/UART 错误为 0。
3. 再看 `app_remote_status()` / 快照：在线、三档模式、四轴与使能/复位是否符合操作。
4. 看 `imu_task_get_report()`：`state`、`faults`、`sensor_result`、`temperature_c`/
   `target_temperature_c`/`heater_duty`、`samples/errors`、`calibration_samples`、
   `gyro_bias_rad_s[]` 与 `last_raw`；再对照 `app_attitude_get_snapshot()` 的 valid/角度/角速度。
   `app_task_get_report()` 也带 `attitude_valid`/`attitude_ms`、`imu_status`/
   `imu_calibration_samples`/`imu_samples`、`heater_duty`/`heater_stale`，无需暂停任务即可观察。
5. 确认 `Motor` 模块未注册任何设备（`app_motor_registered_count() == 0`）。
6. 逐项确认配置后观察状态是否按 `WAIT_CONFIG → WAIT_INPUT → WAIT_DEVICE`
   迁移；不要跳过配置直接测电机。

## 负责人与状态

- 负责人：待认领（工程集成 / C5）。
- 已实现：五个任务启动、App_Control 周期调度、IMU 独立采集/温控/校准/EKF、加热看门狗、请求失效、状态报告。
- 未实现/待上板：温控参数与加热器整定、控制/IMU 周期实时性验证、任务栈与 CPU 占用实测、遥测上报。

## CAN 启动与撤销

`App_Control` 在队列/信号量创建后调用 `can_bsp_start()`；启动失败置 `APP_FAULT_CAN`。周期内读取 `can_bsp_get_stats()`，已注册设备出现 bus-off/连续发送失败时 raise；输出许可下降沿调用 `can_bsp_flush_tx()`，避免继续发送失联前排队的旧动作。

## 功率/能量链路

App_Control 在 CAN 启动前注册 0x31 receiver/observer；每周期先 `app_power_update()`，再按 `app_power_get_policy()` 决定是否清旧底盘输出。功率源失效、低能量或降额时清旧请求/旧 CAN 队列；恢复后从当前输入重新生成，不重放旧动作。

## 适配层状态

Power/Motor 的最终输出只有在真实驱动适配层调用 `app_motor_set_adapter_ready(true)` 后才可能 `enabled`；当前生产代码不调用该接口，`app_motor_submit()` 返回 `NOT_IMPLEMENTED`。不能把软件策略测试通过当作实车执行能力。
