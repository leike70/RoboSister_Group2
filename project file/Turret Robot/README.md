# 炮台车开发说明

多人开发先读 [工作包与文件分工](../../docs/TEAM_DEVELOPMENT.md)：T1负责云台、T2负责发射，T0整合本车上下档入口。成员分别修改自己模块，不集中在 `app_vehicle.c` 中实现所有功能；负责人可兼任，姓名待认领。

本文件只保留炮台车设备、专有模块、构建和上板差异。两车共享的项目约定、遥控/三档状态和 IMU 说明以 [项目约定](../../docs/PROJECT_REQUIREMENTS.md)、[外设与任务](../../docs/PERIPHERALS_AND_TASKS.md) 和 [文档目录](../../docs/README.md) 为准；规则限制见 [规则要求](../../docs/RULES_REQUIREMENTS.md)。

## 当前状态

2026-11-10（框架收尾 + 遥控接入 + IMU/温控移植）：应用框架、遥控接收链路与 BMI088/姿态解算已完成接入；**尚未烧录、未实车验证，控制参数与机构参数全部未确认**。

- 输入：USART3（PC11 RX / PC10 TX，100 kbps、8E1）+ DMA1 Stream1（NORMAL + IDLE）接收 18 字节 DBUS 帧；`Bsp/remote_uart.c` 在中断只保存帧，`App_Control` 任务调用 `User_Driver/Remote/dbus.c` 解析四轴、S1/S2、鼠标/键盘，`App/Remote` 做映射、在线/失联（42 ms）与一致快照。`Config` 的 remote 段已启用，通道/开关初始值待上板核验。
- IMU：`Bsp/imu_bsp.c` 用 SPI1（PB3/PB4/PA7，模式3、5.25 MHz、轮询）+ PA4/PB0 片选访问 BMI088；独立 `IMU_Acquire` 任务 2 ms 采样，500 个静止样本校准零偏，四元数 EKF 解算后由 `app_attitude_ingest()` 发布，`app_attitude_update()` 在 20 ms 无新姿态时撤销。默认单位阵安装、`vehicle_mounting_verified=false`，PF6/TIM10_CH1 已接入 1 kHz 加热 PWM，上电默认占空比 0，温度有效后闭环；目标/PID/保护参数待上板整定，详见 `User_Driver/IMU/README.md`。
- 模式：中档 `APP_MODE_CHASSIS` 走公共底盘；上档 `APP_MODE_VEHICLE_UP` 与下档 `APP_MODE_VEHICLE_DOWN` 进入本车 `App/Vehicle/app_vehicle.c` 的 `app_vehicle_on_up()` / `app_vehicle_on_down()`。两个入口当前均返回 `NOT_IMPLEMENTED`，不产生运动请求；切档、失联、停用或保护拒绝时清除云台/发射旧目标。
- 任务：保留 CAN1/CAN2 发送、DJI 服务与 2 ms `App_Control`，新增独立 2 ms `IMU_Acquire`，共五个 RTOS 任务；每周期先清旧请求，最终只经 `app_motor_submit()` 唯一提交。
- 构建：本轮源码已重新编译通过（RAM 62496 B / FLASH 93884 B）；接手者按下方“构建”从源码生成自己的 `build/Debug`，共享目录不包含旧 ELF。
- 告警：新增应用、IMU、SPI 与遥控源文件 0 条告警；现有 C++ 驱动仍有初始化顺序与 `offsetof` 告警，详见构建输出。
- 默认行为：配置未确认、无设备反馈 → 保护状态 `WAIT_CONFIG`，电机注册数为 0、
  不使能输出；接收机无帧/超时 → `WAIT_INPUT`；非法帧或 UART 错误立即撤销输入；
  IMU 校准/失效期间姿态快照无效。已有 CAN 服务任务保留但 `can_tx_list[].enable` 全为 0，
  不发送报文；未实例化会等待反馈的 `DJI_motor`。
- 串口：USART3 已接 DBUS 接收（100 kbps、8E1、DMA1 Stream1 + IDLE），不能把裸 DBUS
  当普通 TTL UART 直连；USART1（PA9/PB7）仍为 115200/8N1 备用，不接遥控。
- 回归：`tests/run-regressions.ps1` 在本机 CLion MinGW GCC 15.2.0 下 **12/12 组通过**，
  含真实 DBUS 帧路径、本车真实 BMI088/EKF/校准/姿态、SPI BSP 帧格式与 DWT 回绕、
  温控 PWM/保护/看门狗、Gimbal/Shooting/Vehicle、临界区实现。
- 未验证：实车通信、控制闭环、云台/发射动作、达妙 6006 协议、USART3 电平/帧周期/开关字段；
  IMU 轴向/安装矩阵、零偏效果、实际采样周期、SPI 波形、加热器/PF6 波形与温控整定、任务栈与 CPU 占用未测量。
- 注意：遥控/姿态在线不等于车辆可运动；电机/机构配置仍未确认，上下档入口仍返回 `NOT_IMPLEMENTED`。
- 未接入：达妙 6006 驱动、电机硬件适配层（IMU 已接入，等待上板标定）。

## 本车设备

| 逻辑机构 | 电机 | 数量 |
| --- | --- | --- |
| 左/右履带 | 3508 | 2 |
| 前/后抬升 | 6020 | 2 |
| 云台 Yaw | 6020 | 1 |
| 云台 Pitch | 6006 | 1 |
| 两侧摩擦轮 | 3508 | 2 |
| 拨弹 | 2006 | 1 |

合计 9 个电机；CAN 总线、实际 ID、方向、减速比与零位待确认。Pitch 按用户文字需求采用 6006，不能使用 DJI 驱动代替其达妙协议适配。

## 本车规划模块

公共模块覆盖逻辑输入、履带、抬升、姿态、功率、保护与统一电机输出；本车专有部分在 `App/Gimbal/`、`App/Shooting/`，`App/Vehicle/` 是唯一接入层。

姿态链路已接入：`Bsp/imu_bsp.*`（SPI1 轮询/片选/DWT）→ `User_Driver/IMU/`（BMI088、固定存储 EKF、静止校准）→ `Task/imu_task.*`（独立 2 ms 任务）→ `App/Attitude/`（20 ms 超时快照）。默认输出板上传感器坐标，车体安装矩阵未标定，不能直接当作车体姿态使用；PF6/TIM10_CH1 温控已接入但参数待上板整定；yaw 会漂移。

已建立的本车专有模块：

| 文件 | 当前实现 | 默认状态 |
| --- | --- | --- |
| `App/Gimbal/app_gimbal.c` | 保存 Yaw/Pitch 角度目标，对配置齐全的轴写 `POSITION_RAD` 请求；不写速度环/解算/前馈/反陀螺 | `UNCONFIGURED`，反馈接口 `NOT_IMPLEMENTED` |
| `App/Shooting/app_shooting.c` | 摩擦轮 `SPEED_RAD_S` 请求（成对校验）；拨弹转速来源未确认，只记录发射请求、不写占位转速 | `UNCONFIGURED`，卡弹恒为 `UNKNOWN` |
| `App/Vehicle/app_vehicle.c` | 中档不接管底盘并清空本车目标；上/下档分别调用 `app_vehicle_on_up()` / `app_vehicle_on_down()`（当前均 `NOT_IMPLEMENTED`）；切档、失联、停用、保护拒绝时清除云台/发射目标 | `NOT_READY` |
| `Config/app_config.c` | 9 台电机与各机构配置表；`vehicle_kind=TURRET`，全部 `configured=false` | 整体 `UNCONFIGURED` |

云台支持独立两轴控制；摩擦轮需要转速闭环；拨弹需要防卡弹逻辑。目标转速、机械限位、拨弹步进量、卡弹判据和恢复策略尚待确定，首轮只建接口，不填入经验值自动发射。待确认项与逐项启用方法见各模块 README 和 `Config/README.md`。

## 开发验收顺序

1. 无动作骨架可编译，未配置设备不注册、不等待硬件反馈。
2. 核验每个电机的协议、总线、ID、组合帧字节槽和反馈，尤其检查多组 3508/6020 与达妙报文是否冲突。
3. 独立验收底盘、抬升和云台，标定限位与零位。
4. 独立验证两只摩擦轮方向、反馈、转速控制，再验证拨弹及卡弹处理。
5. 最后联调发射条件、模式切换与失联行为，再与工程车验证补给配合。

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

公共模块修改须同步到工程车对应模块；硬件映射及专有控制差异保留在本工程内。模块负责人、已验证参数和后续调试记录在实现时补充，当前均未认领。
