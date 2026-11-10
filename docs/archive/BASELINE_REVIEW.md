# 基础工程检查记录

> 历史记录：本文保留对应阶段的事实与任务。当前开发从 [文档目录](../README.md) 开始；接口状态见 [当前遥控与任务说明](../PERIPHERALS_AND_TASKS.md)。旧文提到的根README详细章节已移到 [项目约定](../PROJECT_REQUIREMENTS.md)。

检查日期：2026-11-10。范围是两车工程源代码、构建配置和姿态参考工程的依赖；未烧录、未连接车辆，未逐份阅读全部 PDF/PPTX。

## 已核实的事实

- 工程车目录：`project file/Engineering robots/`。
- 炮台车目录：`project file/Turret Robot/`。
- 对工程车的对应文件逐个比对，未发现与炮台车内容不同或缺失的文件（比对忽略构建/缓存目录）。当前可以作为相同基线开展两车开发。
- 主控 `.ioc`：STM32F407IGH6；FreeRTOS 使用 CMSIS-RTOS V2。
- CAN1/CAN2 均配置 1 Mbps；CAN1 使用 FIFO0，CAN2 使用 FIFO1。
- 当前管脚为 CAN1 PD0/PD1、CAN2 PB5/PB6；这是工程配置，不是实车接线确认。
- 现有任务：`can1_sending_service`、`can2_sending_service`、`DJI_motor_service`。`freertos.c` 有弱定义，`Bsp/` 和 `User_Driver/` 有对应实现。
- DJI 驱动覆盖 3508、2006、6020；未见车辆应用、电机实例、达妙驱动或遥控接入。
- CMake 已接入 C++ 文件，工具链支持 C++；不能仅因 `enable_language(C ASM)` 这一行就判断工程不支持 C++，实际 `project()` 已启用默认语言。

## 本地构建结果

两车分别从各自工程目录配置，构建目录覆盖到项目根目录的 `work/`：

| 工程 | 配置 | 构建输出目录 | 结果 |
| --- | --- | --- | --- |
| 工程车 | `cmake --preset Debug -B '../../work/baseline-engineer'` | `work/baseline-engineer/` | 42 步构建及 ELF 链接成功 |
| 炮台车 | `cmake --preset Debug -B '../../work/baseline-turret'` | `work/baseline-turret/` | 42 步构建及 ELF 链接成功 |

构建命令在项目根目录执行：

```powershell
cmake --build 'work/baseline-engineer' -j 4
cmake --build 'work/baseline-turret' -j 4
```

工具为本机 STM32CubeCLT：GNU Arm GCC/G++ 14.3.1、CMake、Ninja。受限运行环境最初阻止编译器进程启动；获得本地编译权限后配置及构建成功，不属于工程源代码编译错误。

两车链接输出均为 RAM 56736 B、FLASH 27724 B。存在成员初始化顺序和对非标准布局类型使用 `offsetof` 的警告；没有把警告隐藏或判作运行验证成功。

## 接入电机与整车前的具体问题

以下来自静态代码检查，尚未在板上复现。本轮只记录，不修改原驱动。

### 1. 电机构造函数可能无限等待

`User_Driver/Motor_Driver/DJI_motor_driver.cpp` 的构造函数含 `while (init_offset_f != 1) { osDelay(1); }`，需收到超过 50 帧反馈才退出。设备不在线会一直停留；调度器启动前构造该对象也不符合其延时依赖。

后续应拆成非阻塞初始化与就绪检查，或提供有超时且能报告失败的明确初始化阶段。无硬件框架不创建该对象，不能通过伪造反馈就绪来绕过。

### 2. CAN 接收先解引用再判空

`Bsp/CAN_bsp.cpp` 的两个接收回调先从列表取得 `dev`，随即执行 `memcpy(dev->rx_data, ...)`，之后才判断 `dev`。其 `else` 也访问 `dev->is_callback_none`。应先校验 HAL 接收结果、索引范围及设备指针，再访问成员；依据设备协议检查 DLC、帧类型与回调注册状态。

CAN2 使用过滤器组 14 起的配置，`FilterMatchIndex` 如何映射到本地设备表还需核验，不能凭假设更改偏移。

### 3. CAN 发送队列只复制描述符

`can_device_transmit_member` 保存 `buf_data` 指针。入队后不会复制 8 字节载荷，调用方若传入短生命周期缓存，或者发送前再次改写载荷，就可能发送失效或不一致的数据。`can_send_msg()` 当前向队列传入 `this`，还隐含依赖对象首成员的布局。

后续应明确消息所有权和数据有效期，优先采用带独立载荷的队列消息；若保留共享缓存，则定义同步和读取时机。发送失败、队列满与信号量恢复也须处理，目前服务未检查 `HAL_CAN_AddTxMessage` 的结果。

### 4. 启动顺序与回调资源依赖

`main.c` 在 `osKernelInitialize()` 和 `MX_FREERTOS_Init()` 前启动 CAN 及其中断通知；相关发送完成回调会使用 RTOS 信号量。当前没有整车设备注册，不应据此认为后续接入自然安全。添加设备时明确队列、信号量、设备表与中断使能的先后顺序，并检查 HAL/RTOS 返回状态。

### 5. 组合帧、旧指令与唯一输出者

DJI 服务周期发送已启用的组合帧缓存。初始化后如上层停止刷新，缓存不会自动失效。后续必须定义遥控失联、模式切换、设备掉线与故障时的命令清除/禁用行为。

同一总线中不同 DJI 型号的组合帧和反馈报文可能重叠，不能只按“电机 ID 不同”判断无冲突。设备配置须检查总线、报文 ID 与帧内字节槽；达妙协议也纳入同一张表核验。

### 6. 旧接口的单位与默认参数

现有设定接口中速度和输出采用归一化值，反馈是转子 rpm 和圈数。应用层要转换到机构单位，并使用实际减速比。驱动内置 PID 数值不是本车调参结果，`pid_clear()` 只清积分项，不能假定它能完整清除全部控制历史。

### 7. 姿态参考工程不能直接整套覆盖

参考工程使用 BMI088、DWT、四元数 EKF、温控 PWM 和 CMSIS-DSP 等组件；其任务创建使用 CMSIS-RTOS V1 风格，车辆工程使用 V2。后续须逐项迁移接口、外设和数学库依赖，保留原作者信息。参考工程自带 ELF/HEX 不代表已经适配两车。

## 首轮工作的边界

先完成文档与无动作骨架；电机驱动整改单独列出文件、原因和验收办法。后续 AI 若必须修正影响框架接入的问题，应进行最小范围修改并同步两车，报告每项修改及未验证事项，不能把整个参考工程覆盖过去。
