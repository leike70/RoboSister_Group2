# User_Driver/IMU：BMI088 驱动与姿态解算（两车公共）

## 来源与适配

移植自 `资料文件/陀螺仪移植文件/`（Wang Hongxi 的 BMI088 驱动、QuaternionEKF、
kalman_filter、ins_task 思路）。本轮只取所需源码，**未整套覆盖**参考工程，也未使用
参考工程其他板子的固定零偏。

主要适配：

- 去掉动态分配和 CMSIS-DSP 依赖：矩阵改为 `imu_matrix.*` 的固定上限（6x6）便携实现，
  Kalman/EKF 矩阵全部放在静态结构体内，无堆分配。
- SPI 传输改为带返回值的轮询实现（`Bsp/imu_bsp.*`），加速度计读保留 dummy 字节、
  陀螺仪读不加 dummy，片选在成功/失败后都释放，单次传输有 1 ms 超时。
- 初始化增加 ID/寄存器写后回读检查；先对两颗 die 各做一次读以进入 SPI 模式再软复位。
- 不使用参考工程的外板零偏：启动后由本车静止采集 500 样本求零偏；移动时重新累计。
- 参考 EKF 的卡方检验与渐消因子保留；增加矩阵错误标志、NaN/范围检查和重置路径。
- 采集/解算拆到独立 `IMU_Acquire` 任务，初始化等待用 `osDelay` 让出 CPU，不阻塞 App_Control。
- 输出统一为 rad 与 rad/s；默认板上传感器坐标，安装矩阵在 `Config/imu_config.*` 中配置。
- 增加 BMI088 温控链路：`imu_temperature.c`（PID/抗饱和/预热/保护）、`Bsp/imu_heater.c`
  （PF6/TIM10_CH1 PWM 与看门狗）；参考工程只采用初始 40 °C 与 PID 量级，未直接沿用外板参数。

## 文件与接口

| 文件 | 内容 |
| --- | --- |
| `bmi088.c/.h` | 初始化、寄存器配置/回读、DRDY 判断、LSB→m/s² / rad/s、温度、错误码 |
| `imu_temperature.c/.h` | 温度目标/PID、积分抗饱和、预热就绪、过温/升温异常/过期保护与故障状态 |
| `imu_fusion.c/.h` | 安装矩阵、静止校准、零偏、dt/回绕、EKF 调用、姿态/角速度输出与数值检查 |
| `imu_matrix.c/.h` | 固定容量矩阵：加/减/乘/转置/高斯-约当求逆、粘性错误标志 |
| `kalman_filter.c/.h` | 参考 KF 的固定存储移植（6 状态、0 控制、3 量测） |
| `QuaternionEKF.c/.h` | 参考四元数 EKF、卡方检验、渐消因子；无动态分配 |
| `BMI088reg.h` | 寄存器与位定义 |
| `../../Bsp/imu_bsp.c` | SPI1/CS/DWT 板级适配 |
| `../../Bsp/imu_heater.c` | PF6/TIM10_CH1 PWM、占空比限幅、停更看门狗 |
| `../../Core/Src/tim.c` | TIM10 1 kHz PWM 初始化（PF6/AF3/高有效） |
| `../../Task/imu_task.c` | 独立 2 ms 采集/温控/校准/发布状态机 |
| `../../App/Attitude/` | 对外 20 ms 超时快照 |

关键接口（详见各头文件）：

| 接口 | 说明 |
| --- | --- |
| `bmi088_initialize(bus)` | 有限步初始化；ID/配置错误返回对应 `bmi088_result_t` |
| `bmi088_sample(bus, out)` | 读状态/数据；`BMI088_OK` 时输出 m/s²、rad/s、°C 与时间字段（时间由调用者填） |
| `imu_temperature_update/supervise/abort` | 温度闭环与保护；返回目标、占空比、就绪和故障位 |
| `imu_heater_init/set_duty/stop/supervise` | 硬件 PWM 与 App_Control 看门狗 |
| `imu_fusion_push(sample, out)` | 校准中返回 NOT_READY；完成且 EKF 正常时输出 rad/rad/s 与 valid |
| `imu_fusion_reset()` / `bias()` / `calibration_count()` | 重置、读取零偏与校准进度 |
| `app_attitude_ingest()` / `get_snapshot()` | 姿态快照发布与读取（App/Attitude） |

## 单位与配置

- 加速度计：6 g 量程，常数取参考工程 `0.00179443359375 m/s²/LSB`（按 9.8 换算）。
- 陀螺仪：2000 dps，`0.0010652644360316953 rad/s/LSB`。
- 温度：0.125 °C/LSB，23 °C 偏移；对外 °C。
- `Config/imu_config.c`：采样周期 2 ms、校准 500 样本、超时 20 ms、EKF 参数
  `Q1=10, Q2=0.001, R=1e7, lambda=1, LPF=0`（参考工程初始值）；
  `sensor_to_body` 默认单位阵，`vehicle_mounting_verified=false`。
- 温控初始值：目标 40 °C（参考 `RefTemp`）、PWM 上限 80%（参考 PID 上限为 100%，本项目保守留余量）、
  Kp/Ki/Kd = 0.5/0.01/0（参考 1000/20/0 按 2000 归一化）、积分限幅 0.15（参考 300/2000）；
  就绪 ±1 °C 持续 3 s、预热 120 s、温度过期 300 ms、过温 50 °C、目标+3 °C 持续 5 s 判升温异常（加热开启时）、
  校准/运行容差 2/3 °C、校准超时 60 s。保护阈值和热参数均为本轮新增，待上板整定。

## 状态流程

```text
INIT → TEMP_PREP（有效温度后闭环加热；目标±1 °C 持续 3 s）
     → CALIBRATING（静止 500 样本；移动重新累计；温度偏离 >2 °C 回 TEMP_PREP）
     → RUNNING（发布姿态；温度偏离 >3 °C 回 TEMP_PREP）
     → 故障/重初始化 → FAULT（立即关热、清积分、撤销姿态）→ 500 ms → INIT
```

温度准备和校准期间不发布可用于控制的有效姿态；`app_attitude` 的 `valid` 只在 RUNNING 为真。
`App_Control` 每周期调用 `imu_heater_supervise(now)`，IMU 任务停止刷新 300 ms 后强制占空比 0。

**坐标约定**：当前默认输出**板上传感器坐标**（`sensor_to_body` 单位阵，
`vehicle_mounting_verified=false`）。只有上板核对三轴方向并填写安装矩阵后，才能称“车体坐标姿态”；
六轴 IMU 的 yaw 仍会漂移，不能承诺无漂移绝对航向。

## 调用顺序

`imu_task` → `imu_bsp_init` → `bmi088_initialize` → 2 ms 循环 `bmi088_sample` →
`imu_fusion_push` → `app_attitude_ingest`；App_Control 读取快照。
单写者：融合/EKF 只由 IMU 任务调用；浮点状态不跨任务共享。

## 故障与默认行为

- ID/配置回读/总线错误：`bmi088_initialize` 返回非 OK，500 ms 后重试；期间姿态无效。
- 温度无效、过期、过温、升温异常、预热超时或配置非法：`imu_temperature` 立即输出 duty=0、
  置对应故障位；`imu_task` 关闭 PWM、清积分并进入 FAULT。
- 无 DRDY 超过 20 ms（姿态）/300 ms（温控）、采样错误或数值/矩阵检查失败：撤销姿态并重新初始化。
- 矩阵求逆奇异、维度不符或出现 NaN/Inf：`imu_matrix_failed()` 置位，融合层拒绝本帧并重置。
- 六轴 IMU 的 yaw 会漂移；温控可保持传感器温度，但不提供绝对航向，也不补偿温漂建模误差。
- PF6 默认 PWM=0；只有有效温度且温控状态允许时才输出非零占空比。

## 调试步骤

1. 看 `imu_task_get_report()`：`state`、`faults`、`sensor_result`、`temperature_c`、
   `target_temperature_c`、`heater_duty`、`calibration_samples`、`gyro_bias_rad_s[]`、`last_raw`。
2. 静止时温度准备应达到目标附近并保持 3 s；校准进度到 500；移动时应重新累计。
3. 看 `app_attitude_get_snapshot()`：静止 roll/pitch 与实际摆放一致、角速度接近 0；
   停止采样 20 ms 后应 OFFLINE。
4. 用示波器确认 PF6 1 kHz、占空比与 `heater_duty` 一致；人为停 IMU 任务确认
   `app_task_get_report().heater_stale` 置位且占空比归零。
5. 上板再核对三轴方向、安装矩阵、采样周期、SPI 波形、加热器升温和任务栈/CPU。

## 负责人

待认领（C5：IMU 与姿态）。

## 限制与未实现项

- 未烧录/未上板：实际轴向、零偏效果、精度、采样周期、SPI 时序和任务栈均未验证。
- 车体安装方向未标定（默认单位阵、`vehicle_mounting_verified=false`）。
- 温控代码链路完整，但加热器功率、热惯性、目标温度、PWM 极性、PID 与保护阈值尚未实板验收；
  温控不能消除六轴 yaw 漂移，也未实现姿态闭环控制。
- 测试只能证明控制逻辑、PWM 计算与寄存器帧解析，不能替代实板 SPI、加热器与热闭环验证。
