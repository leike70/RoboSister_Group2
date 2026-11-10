# 主机回归测试

项目根运行 `./tests/run-regressions.ps1`，使用已安装的原生 Windows GCC/CMake/Ninja/CTest；支持 `-Compiler` 指定 GCC。
脚本优先使用 `PATH` 中的 `gcc`；未找到时会尝试作者本机的 CLion/Dev-Cpp 路径（仅本机方便）。接收者请把 `gcc` 加入 `PATH`，或用 `-Compiler` 指向自己的 `gcc.exe`：
`powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run-regressions.ps1 -Compiler "C:\path\to\gcc.exe"`。
首次运行会创建 `work/host-regressions/`，输出不向硬件收发。
若本机执行策略禁止直接运行脚本，使用
`powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run-regressions.ps1`。
后续运行会复用 `work/host-regressions/`；需要从当前源码完整重编时，
先执行 `cmake --build work/host-regressions --clean-first`。

当前共 **12 组**测试：

| 组 | 编译的真实源码 | 覆盖内容 |
| --- | --- | --- |
| `engineer_regressions` | 工程车 Remote/Protection/Attitude/Vehicle/Arm/DBUS/UART | 三档输入与路由、上下档空入口、切档清目标、中档隔离、离线/停用、故障恢复、无效姿态、并发快照 |
| `turret_regressions` | 炮台车 Remote/Protection/Attitude/Vehicle/Gimbal/Shooting/DBUS/UART | 同上（本车机构替换为云台/发射） |
| `imu_engineer_regressions` | 工程车真实 BMI088 驱动、矩阵、KF/EKF、融合、imu_config、Attitude | 寄存器初始化/回读错误、dummy 字节与单位换算、DRDY/ID/总线/饱和错误、矩阵求逆/奇异/维度错误、静止校准与移动重累计、初始倾斜、EKF 收敛、yaw 积分、NaN/时间间隔拒绝、时间回绕、20 ms 姿态超时与恢复 |
| `imu_turret_regressions` | 炮台车同名文件（两车公共） | 同上 |
| `critical_mask_regressions` | 两车真实 `app_critical.c` + CMSIS 寄存器桩 | 嵌套临界区、PRIMASK 保存/恢复、屏障 |
| `imu_bsp_regressions` | 工程车真实 `Bsp/imu_bsp.c` + SPI/GPIO/DWT 桩 | 加速度计 dummy/陀螺仪帧、CS 释放、传输上限与超时、DWT 周期计数回绕 |
| `imu_thermal_regressions` | 工程车真实 `User_Driver/IMU/imu_temperature.c` 和 `Bsp/imu_heater.c` + TIM 桩 | 温度目标/PID 上下限与积分抗饱和、预热稳定判据、预热超时、无效/过期温度、过温、持续升温异常、故障/复位、PWM 占空比限幅和 IMU 停更看门狗 |
| `can_dji_regressions` | 工程车真实 `Bsp/CAN_bsp.cpp`、`DJI_motor_driver.cpp`、`pid_driver.cpp`、`motor_driver.cpp` + HAL/RTOS 桩 | CAN1/CAN2 路由、异常帧、空回调、队列载荷生命周期、空队列不空转、ISR 临界区误用、部分启动/bus-off、统计；DJI 构造不等待、不自动输出、DLC 短帧/反馈就绪/超时和显式使能 |
| `production_chain_regressions` | 工程车真实 `Config/app_config.c`、Motor/Power/Protection 实现 + 时间/遥控边界桩 | 未配置路径、功率无效、适配层未就绪时禁止 enabled、逻辑急停锁存/释放、最终输出失效 |
| `power_motor_contract_regressions` | 工程车真实 Motor/Power/Unit 实现 + 测试输入配置 | 0x31 字序/单位/正负值/DLC/错误总线/超时/时间回绕/低能量/超功率；底盘位置不缩放、最终 `current_limit_norm`、底盘与非底盘旧预算隔离、更新中途读取旧完整快照不阻塞 |
| `config_power_engineer_regressions` | 工程车真实 `Config/app_config.c` | 工程车 50 W / 500 J 规则值和来源未确认标记 |
| `config_power_turret_regressions` | 炮台车真实 `Config/app_config.c` | 炮台车 40 W / 400 J 规则值和来源未确认标记 |

边界说明：

- 遥控组由构造的 18 字节 DBUS 帧经 HAL 回调桩进入 `remote_uart_service`；抽象
  `app_remote_publish()` 用例只验证输入快照逻辑，不作为遥控接入完成依据。
- IMU 组的传感器寄存器、时间与 HAL 均为显式桩，测试中的有效配置只存在于测试程序，
  不修改车辆 `Config`；不能证明 SPI 电平、实际采样率、传感器方向/精度或任务栈。
- 温控测试只验证控制/保护逻辑和 PWM 占空比计算，不验证真实加热器功率、热惯性、温度
  闭环效果或 PF6 电气极性；这些必须上板确认。
- 临界区测试用宿主线程序列模型，不能替代板上的中断延迟测量。
- 未烧录、未连接实车；通过测试不等于硬件行为已验证。
- CAN/DJI 测试使用 HAL/RTOS 边界桩；生产 Config 全未配置，有效功率测试配置仅存在于测试输入中，未修改车辆 Config。

旧的 DBUS/UART 实现与专用测试已由当前接收实现取代。
当前说明与验证见 `docs/PERIPHERALS_AND_TASKS.md`。
