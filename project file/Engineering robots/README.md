# 工程车开发说明

多人开发先读 [工作包与文件分工](../../docs/TEAM_DEVELOPMENT.md)：E1负责关节、E2负责运动学、E3负责作业流程，E0整合本车上下档入口。成员分别修改自己模块，不集中在 `app_vehicle.c` 中实现所有功能；负责人可兼任，姓名待认领。

本文件只保留工程车设备、专有模块、构建和上板差异。两车共享的项目约定、遥控/三档状态和 IMU 说明以 [项目约定](../../docs/PROJECT_REQUIREMENTS.md)、[外设与任务](../../docs/PERIPHERALS_AND_TASKS.md) 和 [文档目录](../../docs/README.md) 为准；规则限制见 [规则要求](../../docs/RULES_REQUIREMENTS.md)。

## 当前状态

2026-11-10（框架收尾 + 遥控接入 + IMU/温控移植）：应用框架、遥控接收链路与 BMI088/姿态解算已完成接入；**尚未烧录、未实车验证，控制参数与机构参数全部未确认**。

- 输入：USART3（PC11 RX / PC10 TX，100 kbps、8E1）+ DMA1 Stream1（NORMAL + IDLE）接收 18 字节 DBUS 帧；`Bsp/remote_uart.c` 在中断只保存帧，`App_Control` 任务调用 `User_Driver/Remote/dbus.c` 解析四轴、S1/S2、鼠标/键盘，`App/Remote` 做映射、在线/失联（42 ms）与一致快照。`Config` 的 remote 段已启用，通道/开关初始值待上板核验。
- IMU：`Bsp/imu_bsp.c` 用 SPI1（PB3/PB4/PA7，模式3、5.25 MHz、轮询）+ PA4/PB0 片选访问 BMI088；独立 `IMU_Acquire` 任务 2 ms 采样，500 个静止样本校准零偏，四元数 EKF 解算后由 `app_attitude_ingest()` 发布，`app_attitude_update()` 在 20 ms 无新姿态时撤销。默认单位阵安装、`vehicle_mounting_verified=false`，PF6/TIM10_CH1 已接入 1 kHz 加热 PWM，上电默认占空比 0，温度有效后闭环；目标/PID/保护参数待上板整定，详见 `User_Driver/IMU/README.md`。
- 模式：中档 `APP_MODE_CHASSIS` 走公共底盘；上档 `APP_MODE_VEHICLE_UP` 与下档 `APP_MODE_VEHICLE_DOWN` 进入本车 `App/Vehicle/app_vehicle.c` 的 `app_vehicle_on_up()` / `app_vehicle_on_down()`。两个入口当前均返回 `NOT_IMPLEMENTED`，不产生运动请求；切档、失联、停用或保护拒绝时清除机械臂与任务旧目标。
- 任务：保留 CAN1/CAN2 发送、DJI 服务与 2 ms `App_Control`，新增独立 2 ms `IMU_Acquire`，共五个 RTOS 任务；每周期先清旧请求，最终只经 `app_motor_submit()` 唯一提交。
- 构建：本轮源码已重新编译通过（RAM 62536 B / FLASH 94040 B）；接手者按下方“构建”从源码生成自己的 `build/Debug`，共享目录不包含旧 ELF。
- 告警：新增应用、IMU、SPI 与遥控源文件 0 条告警；现有 C++ 驱动仍有初始化顺序与 `offsetof` 告警，详见构建输出。
- 默认行为：配置未确认、无设备反馈 → 保护状态 `WAIT_CONFIG`，电机注册数为 0、
  不使能输出；接收机无帧/超时 → `WAIT_INPUT`；非法帧或 UART 错误立即撤销输入；
  IMU 校准/失效期间姿态快照无效。已有 CAN 服务任务保留但 `can_tx_list[].enable` 全为 0，
  不发送报文；未实例化会等待反馈的 `DJI_motor`。
- 串口：USART3 已接 DBUS 接收（100 kbps、8E1、DMA1 Stream1 + IDLE），不能把裸 DBUS
  当普通 TTL UART 直连；USART1（PA9/PB7）仍为 115200/8N1 备用，不接遥控。
- 回归：`tests/run-regressions.ps1` 在本机 CLion MinGW GCC 15.2.0 下 **12/12 组通过**，
  含真实 DBUS 帧路径、本车真实 BMI088/EKF/校准/姿态、SPI BSP 帧格式与 DWT 回绕、
  温控 PWM/保护/看门狗、Arm/Vehicle、临界区实现。
- 未验证：实车通信、控制闭环、机械臂运动学、机构动作、USART3 电平/帧周期/开关字段；
  IMU 轴向/安装矩阵、零偏效果、实际采样周期、SPI 波形、加热器/PF6 波形与温控整定、任务栈与 CPU 占用未测量。
- 注意：遥控/姿态在线不等于车辆可运动；电机/机构配置仍未确认，上下档入口仍返回 `NOT_IMPLEMENTED`。

## 本车设备

| 逻辑机构 | 电机 | 数量 |
| --- | --- | --- |
| 左/右履带 | 3508 | 2 |
| 前/后抬升 | 6020 | 2 |
| 机械臂 Yaw、大臂 Pitch | 4310 | 2 |
| 小臂 Pitch、爪子 Roll | 3508 | 2 |
| 爪子 Pick | 2006 | 1 |

合计 9 个电机；CAN 总线、实际 ID、方向、减速比与零位待确认。4310 不能使用现有 DJI 驱动代替。

## 本车规划模块

公共模块覆盖输入、履带、抬升、姿态、功率、保护与统一电机输出；专有 `App/Arm/` 覆盖关节目标、运动学接口和任务状态；`App/Vehicle/` 是本车唯一接入层，上/下档分别由 `app_vehicle_on_up()` / `app_vehicle_on_down()` 自行实现。

姿态链路已接入：`Bsp/imu_bsp.*`（SPI1 轮询/片选/DWT）→ `User_Driver/IMU/`（BMI088、固定存储 EKF、静止校准）→ `Task/imu_task.*`（独立 2 ms 任务）→ `App/Attitude/`（20 ms 超时快照）。默认输出板上传感器坐标，车体安装矩阵未标定，不能直接当作车体姿态使用；PF6/TIM10_CH1 温控已接入但参数待上板整定；yaw 会漂移。

机械臂负责抓取飞镖、搬运弹夹、向炮台车倾倒弹丸。当前已实现的是无动作骨架：

- `App/Vehicle/app_vehicle.c`：中档不接管公共底盘并清空本车目标；上/下档入口当前均返回 `NOT_IMPLEMENTED`；切档、失联、停用或保护拒绝时清除关节与任务目标，避免串档。
- `App/Arm/app_arm.c`：保存单关节目标；配置有效且电机运动参数齐全时写 `POSITION_RAD` 请求并限幅到已确认限位；配置无效时不写任何请求、不伪造反馈。
- `App/Arm/app_arm_kinematics.c`：正解/逆解一律返回 `NOT_IMPLEMENTED`，输出 `valid=false` 或全零。
- `App/Arm/app_arm_task.c`：合法任务请求置 `UNSUPPORTED` 并返回 `NOT_IMPLEMENTED`，永不报告 `COMPLETED`。

当前连杆、坐标系、限位未确认，求解接口须返回未就绪，不能输出虚构关节角或自动完成动作。各模块默认状态、调试步骤与待确认项见对应目录的 `README.md`。

## 开发验收顺序

1. 无动作骨架可编译，未配置设备不注册、不等待硬件反馈。
2. 确认每个电机的协议、总线、ID、方向、反馈与异常状态。
3. 验证机械零位、行程与关节限位；独立验收底盘和抬升。
4. 先单关节，再关节联动；建立正运动学验证，再实现逆解可达性和约束处理。
5. 最后逐步验证抓取、搬运与倾倒，结合夹持力及支撑条件确定故障策略。

## 构建

需要 `arm-none-eabi-gcc` / `arm-none-eabi-g++` 在 `PATH`，并安装 CMake ≥ 3.22、Ninja。在本目录执行（输出在 `build/Debug/`，已被 `.gitignore` 排除）：

```powershell
cmake --preset Debug -B build/Debug
cmake --build build/Debug -j 4
```

输出 ELF：`build/Debug/dx_assignment5_freertos.elf`。两车 ELF 同名，烧录前必须按工程目录确认固件来源。

## 烧录与调试（本轮未实际执行）

本机使用 DAP-Link + OpenOCD，工程内已有 `daplink_f4.cfg`（CMSIS-DAP/SWD/STM32F4）。确认目标板供电和 SWD 接线后，可在本工程目录执行：

```powershell
openocd -f daplink_f4.cfg -c "program build/Debug/dx_assignment5_freertos.elf verify reset exit"
```

若探针或 OpenOCD 版本不同，按实际接口调整；不要在两车之间混用同名 ELF。烧录前先确认工程目录和 ELF 来源。**本轮没有连接硬件，以上仅为可执行步骤，不代表已经烧录或验证。**

公共模块修改须同步到炮台车对应模块；硬件映射及专有控制差异保留在本工程内。模块负责人、已验证参数和后续调试记录在实现时补充，当前均未认领。
