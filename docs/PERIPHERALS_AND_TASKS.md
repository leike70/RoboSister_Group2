# 两车中档、上档、下档入口、遥控与 IMU

更新日期：2026-11-10（IMU 移植复核）。两车各一块 C 板、两个独立工程；公共逻辑在两车内各保留一份。

## 三个入口

| 档位 | 公共逻辑 | 工程车自行修改 | 炮台车自行修改 |
| --- | --- | --- | --- |
| 中档 `APP_MODE_CHASSIS` | `Task/app_task.c` 调公共 Chassis 模块 | 本车尺寸、方向等放 Config | 本车尺寸、方向等放 Config |
| 上档 `APP_MODE_VEHICLE_UP` | 只做保护与分派 | 本车 `App/Vehicle/app_vehicle.c` 的 `app_vehicle_on_up()` | 本车同名文件的 `app_vehicle_on_up()` |
| 下档 `APP_MODE_VEHICLE_DOWN` | 只做保护与分派 | 本车同名文件的 `app_vehicle_on_down()` | 本车同名文件的 `app_vehicle_on_down()` |

每辆车各有两个独立的上下档函数，不调用另一辆车源码。工程车函数体后续调用 Arm 等本车模块；炮台车调用 Gimbal/Shooting 等本车模块。**当前四个函数均返回 NOT_IMPLEMENTED，不生成运动请求**；上下档不预设为手动/自动。

## 调用边界

两车保留各自一块 C 板、独立工程，以及五个 RTOS 任务：CAN1 发送、CAN2 发送、DJI 服务、2 ms App_Control、2 ms IMU_Acquire。
遥控接收由 USART3/DMA 中断保存原始帧，解析、映射、发布、保护判断与三档分派都在 App_Control 上下文完成，不新增遥控任务。IMU_Acquire 独立负责 BMI088 采样、温控与 EKF，只通过 `app_attitude_ingest()` 发布姿态，不参与三档分派和电机输出。

整车周期统一清除旧请求，再在中档生成底盘请求，并经 Vehicle 层分派本车上下档；最后由 Motor 唯一提交。
Vehicle 层只有在输入在线、保护允许且为上下档时才进入对应函数。上/下档互切、切回中档、失联或停用时，清除此前保存的本车目标/任务，避免串档。

## 遥控硬件与接收链路

| 用途 | 外设/引脚 | 配置 |
| --- | --- | --- |
| DR16/DT7 DBUS | USART3，PC11 RX、PC10 TX | 100 kbps、8 数据位、偶校验、1 停止位；HAL 选 9B+EVEN |
| 接收 DMA | DMA1 Stream1 Channel4 | 外设→内存、字节、NORMAL、高优先级；USART3/DMA NVIC 优先级 5 |
| 调试备用 | USART1，PA9 TX、PB7 RX | 115200、8N1，仅初始化，不接遥控 |
| IMU SPI | SPI1：PB3 SCK、PB4 MISO、PA7 MOSI | 主机、全双工、8bit、模式3（CPOL=1/CPHA=2）、5.25 MHz、软件 NSS、轮询传输、1 ms 超时 |
| IMU 片选 | 加速度计 PA4、陀螺仪 PB0 | 推挽输出，空闲高；传输失败也释放 |
| IMU 加热 | PF6 / TIM10_CH1 | AF3、PWM1、高电平有效、1 kHz（PSC=84-1、ARR=2000-1）；上电默认 0，温控最大占空比 0.8 |

数据流：

```text
USART3 → DMA/IDLE IRQ（保存一帧字节 + 帧到达时刻，并重挂接收）
       → App_Control: remote_uart_service()
       → dbus_decode()（18 字节位域、长度/通道/开关/鼠标键校验）
       → dbus_to_input()（通道下标/方向、死区、档位/使能开关映射）
       → 用“到达时刻”判断是否过期（过期帧丢弃，不发布）
       → app_remote_publish_frame()（逻辑输入 + 原始字段同一临界区发布）
       → app_remote_update()（按到达时刻判在线/失联、一致快照）
       → Protection → 中档 Chassis / 上下档 Vehicle
```

- 协议依据：`资料文件/遥控器/遥控器控制协议V1.4.pdf` 与 DT7/DR16 用户手册。
  DBUS 18 字节/帧；`ch0..ch3` 范围 364–1684、中位 1024；S1/S2 取值 1=上、2=下、3=中；
  鼠标/键盘字段解析后保留，当前不映射为动作。
- 物理信号需要板上 DBUS 接口的反相/电平链路，不能把裸 DBUS 当普通 TTL UART 直连。
- 接收统计：`remote_uart_get_stats()` 提供有效帧、非法帧、过期帧、UART 错误、重启、
  启动失败计数。
- 帧到达时刻由接收回调用 `app_time_now_ms()` 保存，与任务同一时间基准；任务延迟处理
  时按到达时刻判断，过期帧不会重新获得在线窗口（含 32 位时间回绕处理）。

## 通道/开关映射与操作

映射参数在 `Config/app_config.c` 的 `remote` 段，每项有独立有效性标记；当前初始值取协议手册与参考工程，**需上板逐项核验**：

- 轴：前进 ch3、左转 -ch2、功能轴1 -ch0、功能轴2 ch1；死区 0.03。
- 档位开关：S1（左）；使能/复位开关：S2（右）。
- 失联门限：42 ms（参考工程实测帧周期 14 ms 的 3 倍；本车未实测）。

| 操作 | 结果 |
| --- | --- |
| 左 S1 中档 | `APP_MODE_CHASSIS`，走公共底盘 |
| 左 S1 上档 | `APP_MODE_VEHICLE_UP`，进入本车 `app_vehicle_on_up()` |
| 左 S1 下档 | `APP_MODE_VEHICLE_DOWN`，进入本车 `app_vehicle_on_down()` |
| 右 S2 上档 | 请求使能（`arm_request`）；使能仍需通过配置、设备与保护条件 |
| 右 S2 中档 | 停用（不请求使能） |
| 右 S2 下档 | 停用，同时产生故障复位请求 |

这是本方案的简单电平使能规则，**不复用旧工程“下档回中后上档”的重连资格序列**。非法帧、UART 错误或超时会立即撤销输入，恢复在线后使能只看当前开关状态。

## 原始状态快照（供上下档入口读取）

`App/Remote` 除逻辑快照外，还保存同一帧的原始/协议字段，供各车 `app_vehicle_on_up()` /
`app_vehicle_on_down()` 自行读取，不绑定任何机构动作：

| 接口 | 字段 |
| --- | --- |
| `app_remote_get_aux_snapshot(&aux)` | `valid`、`timestamp_ms`（帧到达时刻）、`channel[4]`、`switch_s1`、`switch_s2`、`mouse_x/y/z`、`mouse_press_left/right`、`keyboard` |

- 逻辑输入与原始字段由 `app_remote_publish_frame()` 在同一临界区发布，保证同帧一致。
- `valid=false` 时所有字段清零、时间戳为 `APP_TIME_NEVER`；失联、超时、非法帧或 UART
  错误都会清空，避免残留按下状态。
- 只提供状态，不规定按键/鼠标用途；各车负责人自行决定如何使用。

## IMU 采集与姿态（BMI088 + 四元数 EKF）

移植来源：`资料文件/陀螺仪移植文件/`（Wang Hongxi BMI088 驱动、QuaternionEKF、kalman_filter）。
适配修改：去掉动态分配与 DSP 依赖，矩阵改为固定上限的便携实现；所有 SPI 传输带返回值检查、
片选失败也释放；不使用参考工程其他板子的固定零偏。

数据流：

```text
IMU_Acquire（2 ms）→ Bsp/imu_bsp（SPI1 轮询、CS、DWT 微秒时间戳）
  → bmi088_initialize/sample（ID/配置回读、DRDY、LSB→m/s²、LSB→rad/s、温度）
  → imu_fusion（安装矩阵、静止校准、陀螺零偏、dt/回绕、数值检查）
  → QuaternionEKF（固定存储、卡方检验、协方差下限）
  → app_attitude_ingest（有效/时间戳）→ App_Control 读 app_attitude_get_snapshot()
```

- 配置在 `Config/imu_config.*`：采样周期 2 ms、静止校准 500 样本、姿态超时 20 ms、
  安装矩阵默认单位阵且 `vehicle_mounting_verified=false`、EKF 参数取参考工程初始值。
- 启动/重初始化在 IMU 任务内用 `osDelay` 让出 CPU；校准完成前不发布有效姿态。
  移动（角速度或加速度变化超阈值）会清零重新累计，不使用外板固定零偏。
- 对外单位：roll/pitch/yaw 为 rad，角速度为 rad/s；超时后 `app_attitude_update()`
  清零并置 OFFLINE；`app_attitude_get_snapshot()` 同时给 `valid` 与毫秒时间戳。
- 六轴 IMU 的 yaw 会漂移，不能作为无漂移绝对航向。
- 复核修复：陀螺仪软复位前先读一次 CHIP_ID 进入 SPI 模式，避免复位写被忽略；固定存储中
  KF 控制向量槽位与 `xhat_data` 解耦；`app_attitude_get_snapshot()`、`imu_task_get_report()`、
  `app_task_get_report()` 保留在有生产调用的路径上，调试器可直接读取。
- 调试：`imu_task_get_report()` 看传感器结果、状态、样本数、错误数、初始化次数、校准进度、
  `gyro_bias_rad_s[]`、温度/目标温度/加热占空比/温度故障、最近原始样本；
  `app_attitude_get_snapshot()` 看姿态与有效性；`app_task_get_report()` 还提供姿态和 IMU 汇总。

## IMU 温控（PF6 / TIM10_CH1）

**硬件依据**：参考工程 `Src/tim.c` 明确给出 PF6→TIM10_CH1(AF3)、TIM_OCMODE_PWM1、
`OCPolarity=TIM_OCPOLARITY_HIGH`、`Prescaler=84-1`、`Period=2000-1`；TIM10 挂 APB2，
按 168 MHz 系统时钟计数 168 MHz/84=2 MHz，再 /2000 得 1 kHz。参考 `ins_task.c` 使用
`RefTemp=40`、`HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1)`，PID 初值
`max_out=2000, integral_limit=300, Kp=1000, Ki=20, Kd=0`。这些只作为本板初始依据，
**未在本车加热器上验证**，不得当作已整定参数。

**代码链路**：

```text
IMU_Acquire 每 2 ms 采样 BMI088 温度
  → User_Driver/IMU/imu_temperature.c（PID、抗饱和、就绪/预热/保护状态）
  → Bsp/imu_heater.c（TIM10 CCR、0/上限限幅、停更看门狗）
  → TIM10_CH1 / PF6；App_Control 每周期调用 imu_heater_supervise()
```

**状态流程**（`imu_task` 内部，非阻塞）：

```text
INIT（外设/传感器初始化）
  → TEMP_PREP（温度有效才允许加热，PID 闭环；达到“目标±1°C 持续 3 s”）
  → CALIBRATING（静止 500 样本；移动重新累计；温度偏离目标 >2°C 回 TEMP_PREP）
  → RUNNING（EKF 姿态发布；温度偏离 >3°C 回 TEMP_PREP）
  → 任一故障/重初始化 → FAULT（立即关加热、撤销姿态）→ 500 ms 后 INIT
```

**保护与配置**（`Config/imu_config.c` 的 `temperature` 段，全部待上板确认）：

| 项 | 当前初值 | 依据/说明 |
| --- | --- | --- |
| 目标温度 | 40 °C | 参考工程 `RefTemp=40`；本板热平衡未知 |
| PWM 频率/上限 | 1 kHz / 80% | 参考 TIM10 配置；参考 PID 上限 100%，本项目先留 20% 保守余量 |
| Kp / Ki / Kd | 0.5 / 0.01 / 0 | 由参考 CCR 值按 2000 归一化（1000/2000、20/2000） |
| 积分限幅 | 0.15 占空比 | 参考 300/2000；另有饱和回算抗饱和 |
| 控制周期 | 100 ms | 热惯性远慢于 IMU；避免每 2 ms 重算 |
| 就绪判据 | ±1 °C 持续 3 s | 新加，防止未稳定就校准 |
| 预热超时 | 120 s | 新加；超时置 PREHEAT_TIMEOUT 并停热 |
| 温度过期 | 300 ms | 新加；超时停热并置 TEMP_STALE |
| 过温/升温异常 | 50 °C；目标+3 °C 持续 5 s（加热开启时） | 新加；置 OVERTEMP/RUNAWAY 并停热 |
| 有效范围 | -20…60 °C | 新加；超范围置 TEMP_INVALID |
| 校准/运行容差 | 2 °C / 3 °C | 新加；超出时退出校准或回到 TEMP_PREP |
| 校准超时 | 60 s | 新加；持续移动/无法完成时置 CALIBRATION_TIMEOUT |

- 上电、故障和重初始化时 `imu_heater_stop()` 把 CCR 清 0，温控积分清零；只有收到有效
  温度后才允许输出非零占空比。
- 温度任务停更保护：`App_Control` 每周期调用 `imu_heater_supervise(now_ms)`；非零占空比
  超过 300 ms 未刷新即强制 CCR=0，并置 `heater_stale`。该监督不依赖 IMU 任务继续运行。
- 温度准备/校准期间不发布有效姿态；`app_attitude` 保持无效，App_Control、遥控和 CAN
  服务不被阻塞。
- 任务与栈：`IMU_Acquire` 优先级 Normal、栈 8192 B、目标周期 2 ms；`App_Control`
  AboveNormal、栈 2048 B、周期 2 ms。仍是 5 个任务，未新增温控线程；实际栈高水位/CPU 待测。

## 已移除与保留的资源

旧遥控链路曾整体撤下，随后按上述新约定重新接入 DBUS 接收（协议解析与映射代码为新实现，模式与使能语义按本文）。旧版恢复副本原保存在本机 `work/remote-chain-before-removal/`；共享目录不包含该临时备份，开发应以当前两车源码和本节接口为准。

USART1 保持通用 115200、8N1 初始化，仅作为资源预留；没有日志输出或调试命令任务。

## 2026-11-10 基础通信与保护加固

- CAN 启动移到 App_Control，队列/信号量就绪后执行 `can_bsp_start()`；启动失败置 `APP_FAULT_CAN`。
- 接收按“已校验 CAN 实例 + 标准 ID”查注册设备，不依赖 `FilterMatchIndex`；发送队列保存独立载荷副本。
- 已注册设备出现 bus-off/连续发送失败时，App_Control 汇总为 CAN 故障；输出许可下降沿/模式切换时 `can_bsp_flush_tx()` 撤销旧动作。
- 保护新增逻辑急停和 `APP_FAULT_CAN`；现行底盘功率策略只门控左右履带，非底盘机构沿用独立代理预算。位置目标不做功率缩放，电机适配未接时最终输出均禁用。
- 遥控配置新增 `hardware_verified=false`：在线/可诊断不等于允许运动，必须上板核验后才进入运动许可。
- 当前主机回归共 12 组；包括 CAN/DJI、真实 Config/Motor/Power/Protection 和功率/电机合同测试。详见 `tests/README.md` 与 `docs/CHANGELOG.md`。

## 验证结果（2026-11-10 实际执行）

两车已从当前源码重新编译，退出码均为 0。接手者可从各自工程目录重新生成输出：

```powershell
cd "project file/Engineering robots"
cmake --preset Debug -B build/Debug
cmake --build build/Debug -j 4
# 工程车：RAM 62536 B / FLASH 94040 B

cd "project file/Turret Robot"
cmake --preset Debug -B build/Debug
cmake --build build/Debug -j 4
# 炮台车：RAM 62496 B / FLASH 93884 B
```

- ELF 由接手者分别在两车 `build/Debug/` 中生成；共享目录不含本机 ELF。
- 两车构建为 exit 0；既有 DJI/pid 驱动仍有 `-Wreorder`/`-Winvalid-offsetof` 告警，未新增应用逻辑告警。
- ELF 证据：`remote_uart_init/service`、`dbus_decode`、`dbus_to_input`、
  `app_remote_publish_frame`、`app_remote_get_aux_snapshot`、`USART3_IRQHandler`、
  `DMA1_Stream1_IRQHandler`、`hdma_usart3_rx` 均存在；接收回调反汇编可见
  `app_time_now_ms` 调用，`remote_uart_service` 反汇编可见
  `dbus_decode → dbus_to_input → app_remote_publish_frame` 调用。IMU 侧
  `imu_task`、`bmi088` 驱动、`imu_fusion_push`、`IMU_QuaternionEKF_Update`、
  `app_attitude_get_snapshot`、`MX_SPI1_Init`、`HAL_SPI_TransmitReceive`、
  `MX_TIM10_Init`、`HAL_TIM_PWM_Start`、`imu_heater_init`、`imu_temperature_update` 均存在；
  `imu_task_get_report`、`app_task_get_report`、`app_attitude_get_snapshot` 保留在
  ELF 调试符号中，可直接在调试器调用；`freertos.c` 创建 5 个线程。
  仍无 `DJI_motor` 构造函数符号，默认不注册电机。
- 主机回归（`tests/run-regressions.ps1`，CLion MinGW GCC 15.2.0）**12/12 组通过**：
  两车框架/DBUS 路径 2 组、两车真实 IMU 驱动/EKF/校准/姿态 2 组、临界区 1 组、
  IMU BSP 1 组、温控 1 组、CAN/DJI 1 组、真实 Config/Motor/Power/Protection 1 组、功率/电机合同 1 组、两车功率/能量配置 2 组。IMU 回归覆盖：初始化寄存器与回读错误、
  加速度计 dummy 字节与单位换算、DRDY/ID/总线/饱和错误、矩阵求逆/奇异/维度错误、
  静止校准与移动重累计、初始倾斜、EKF 收敛、yaw 转角积分、NaN/时间间隔拒绝、
  32 位时间回绕、`app_attitude` 20 ms 超时与恢复。
  历史遥控复核曾在本机临时目录执行独立 probe；共享目录只保留可复跑的 `tests/` 回归测试。
  抽象 `app_remote_publish()` 测试保留，但**不作为遥控接入完成依据**。

- 源码/HAL/.ioc 配套：HAL SPI/TIM 模块与 `stm32f4xx_hal_spi.c`/`stm32f4xx_hal_tim.c` 参与构建，
  `Core/Src/spi.c`/`Core/Src/tim.c` 在 `cmake/stm32cubemx/CMakeLists.txt` 的源文件列表中；`.ioc` 的 SPI1 引脚与模式3/预分频16
  与 `spi.c` 一致；`main.c` 调用 `MX_SPI1_Init()` 和 `MX_TIM10_Init()`，`tim.c` 中
  PF6→TIM10_CH1(AF3)、PWM1/高有效、PSC=84-1、ARR=2000-1 与 `.ioc` 一致；
  `freertos.c` 创建 5 个线程；
  轮询传输不启用 SPI 中断，`.ioc` 没有 SPI IRQ；TIM10 仅作 PWM，不启用更新中断。
- 两车公共 IMU/SPI/温控文件（Bsp/imu_bsp、Bsp/imu_heater、User_Driver/IMU、Config/imu_config、
  Task/imu_task、App/Attitude、Core/Spi、Core/tim）逐文件比对一致；只有 `Config/app_config.c` 的电机表和各车专有模块不同。

未执行：烧录、上板、USART3 电平/帧周期/开关字段实测、SPI 波形与 BMI088 实际采样、
IMU 轴向/安装矩阵/零偏效果、加热器功率与 PF6 波形、温控闭环和目标温度、SPI/任务实时性、
任务栈和 CPU 占用测量。默认配置中电机与机构仍未确认，因此遥控/姿态在线不代表车辆能够运动；
上下档入口仍未实现。

## IMU 上板检查步骤

1. 只接 C 板与 IMU（不接电机动力）；PF6 先接示波器或假负载，不直接接真实加热器，
   确认 1 kHz、PWM1、高电平有效，复位后占空比应为 0。
2. 复位后先看 `imu_task_get_report()`：
   - `sensor_result` 应为 `BMI088_OK`；持续 `BAD_ID/BUS_ERROR/CONFIG_ERROR` 说明 SPI/CS/寄存器回读有问题；
   - `state` 应从 INIT → TEMP_PREP；`temperature_valid` 只有在有效温度后为真；
   - `temperature_c`、`target_temperature_c`（当前 40 °C）、`heater_duty`（0..0.8）应合理；
   - `calibration_samples` 静止时从 0 增到 500 后进入 RUNNING，移动时应清零重新累计；
   - `faults`/`temperature_faults` 为 0；出现 `OVERTEMP/RUNAWAY/PREHEAT_TIMEOUT/STALE` 应先关加热排查。
3. 等温度进入“目标 ±1 °C 持续 3 s”后应进入 CALIBRATING；此阶段 `app_attitude_get_snapshot()`
   必须保持 `valid=false`。校准中温度偏离目标超过 2 °C 会回到 TEMP_PREP，不应直接输出姿态。
4. 进入 RUNNING 后看 `app_attitude_get_snapshot()`：`valid=true`、时间戳持续更新；静止时
   roll/pitch 与实际摆放一致、角速度接近 0。温度偏离目标超过 3 °C 应回到 TEMP_PREP 并撤销姿态；
   超过 20 ms 无新姿态应变为 `valid=false`/OFFLINE。
5. 逐项核对轴向与安装矩阵：分别绕三个车体轴缓慢转动，检查对应角速度符号与角度变化；
   当前安装矩阵为单位阵且 `vehicle_mounting_verified=false`，不要据此断言车体姿态已经正确。
6. 记录实际采样周期、`IMU_Acquire` 任务栈高水位、CPU 占用、SPI 波形；测量加热器升温曲线、
   稳态占空比、目标温度保持误差、超调、校准重复性与角度漂移，再整定 `Config/imu_config.c`
   的 `temperature` 参数和保护阈值。
7. 人为断开 IMU、制造温度过期/过温/持续升温，确认 `imu_heater_get_duty()` 变 0、
   `imu_heater_is_stale()` 可观测，故障后 500 ms 重新走 INIT→TEMP_PREP→CALIBRATING。
8. 六轴 IMU 的 yaw 会漂移，静止长时间后必须重新参考；温控已接入但目标/PID 和保护阈值
   尚未在本车加热器上验收。
9. 上板发现的问题写回本文件与 C5 工作包，不改写历史记录；未测项目保持“未上板”。

## 上板第一步

1. 断开电机动力，仅接 C 板与接收机，确认 DBUS 电平链路与波特率/校验。
2. 在调试器看 `remote_uart_get_stats()`：有效帧计数应随接收机周期递增，非法/过期帧为 0。
3. 看 `app_remote_get_snapshot()`：四轴方向、S1 三档、S2 使能/复位逐项核对，
   确认与本文映射一致后再改 `Config/app_config.c`。
4. 看 `app_remote_get_aux_snapshot()`：原始通道、S1/S2、鼠标、键盘与操作一致；
   关控/拔线后应变为 `valid=false` 且字段清零。
5. 关控/拔线确认 42 ms 后离线、输出许可撤销；重新连接确认自动恢复在线。
6. 最后才按两车 README 的验收顺序接电机、标定并验收底盘/机构。
