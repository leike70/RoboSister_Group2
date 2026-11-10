# 首轮应用框架交付记录

> 历史记录：本文保留对应阶段的事实与任务。当前开发从 [文档目录](../README.md) 开始；接口状态见 [当前遥控与任务说明](../PERIPHERALS_AND_TASKS.md)。旧文提到的根README详细章节已移到 [项目约定](../PROJECT_REQUIREMENTS.md)。

日期：2026-11-10。范围：两车工程内的应用层骨架、配置门控、周期调度入口与构建验证。

> 本文保留首轮交付的原始记录。后续 4 项逻辑修复、新增临界区组件及运行测试，
> 以 `docs/archive/FRAMEWORK_FIXES.md` 为准；不再使用下文首轮的文件数与内存占用描述最新版本。
> 2026-11-10 起旧遥控链路已从运行源码撤下，模式改为中档/上档/下档；下文提到的
> `APP_MODE_VEHICLE_MANUAL` / `APP_MODE_VEHICLE_AUTO`、`app_remote_ingest` 与 DR16 接入
> 均已作废。当前接口与分工以 `docs/PERIPHERALS_AND_TASKS.md` 和两车 README 为准。

本轮只交付根 README 第 10 节的**第 1 阶段（框架阶段）**：模块接口与状态、配置有效性门控、实际任务入口、两车独立编译。未烧录、未连车、未做任何控制闭环或机构动作验证。本文中的“已实现”只表示代码与调用路径存在、两车编译通过，不表示实车可用。

## 1. 交付结论

| 项目 | 工程车 | 炮台车 |
| --- | --- | --- |
| 工程目录 | `project file/Engineering robots/` | `project file/Turret Robot/` |
| 新增 C 源文件 | 17 | 17 |
| 新增文件（含头文件/README） | 46 | 45 |
| Debug 配置 | 退出码 0 | 退出码 0 |
| Debug 构建（`--clean-first`） | 退出码 0，60 步 | 退出码 0，59 步 |
| ELF | `work/framework-engineer/dx_assignment5_freertos.elf` | `work/framework-turret/dx_assignment5_freertos.elf` |
| RAM / FLASH | 57696 B / 38164 B | 57656 B / 38532 B |
| 基线 RAM / FLASH | 56736 B / 27724 B | 56736 B / 27724 B |

两车上电默认：配置未确认 → 不注册任何电机、不构造会等待反馈的 `DJI_motor`、不发送任何 CAN 报文、不使能输出。所有专有机构接口返回 `UNCONFIGURED` / `NOT_IMPLEMENTED` / `NOT_READY`，不返回伪造测量或动作完成。

## 2. 新增目录树

工程车：

```text
project file/Engineering robots/
├── App/
│   ├── Common/        app_types.h/.c  app_time.h/.c  README.md
│   ├── Remote/        app_remote.h/.c  README.md
│   ├── Chassis/       app_chassis.h/.c  README.md
│   ├── Lift/          app_lift.h/.c  README.md
│   ├── Attitude/      app_attitude.h/.c  README.md
│   ├── Power/         app_power.h/.c  README.md
│   ├── Protection/    app_protection.h/.c  README.md
│   ├── Motor/         app_motor.h  app_motor_internal.h  app_motor_registry.c
│   │                  app_motor_output.c  app_unit.h/.c  README.md
│   ├── Vehicle/       app_vehicle.h/.c  README.md
│   └── Arm/           app_arm.h/.c  app_arm_kinematics.h/.c  app_arm_task.h/.c  README.md
├── Config/            app_config.h/.c  README.md
└── Task/              app_task.h/.c  README.md
```

炮台车：公共目录与文件同工程车（`App/Common`、`Remote`、`Chassis`、`Lift`、`Attitude`、`Power`、`Protection`、`Motor`、`Vehicle`、`Config`、`Task` 完全一致），专有模块替换为：

```text
project file/Turret Robot/App/
├── Gimbal/            app_gimbal.h/.c  README.md
└── Shooting/          app_shooting.h/.c  README.md
```

修改的既有文件（每车各一份）：`CMakeLists.txt`、`Core/Src/freertos.c`（仅 `USER CODE` 区）、`README.md`。项目根修改：`README.md`；新增本文 `docs/archive/FRAMEWORK.md`。未改动 `Core/Src/main.c`、`.ioc`、链接脚本、启动文件、HAL/FreeRTOS 配置、`Bsp/`、`User_Driver/` 任何既有驱动文件。

## 3. 模块依赖

只读依赖，无初始化环：

```text
Config(app_config.*) ── 所有模块读取配置与有效性标记
Common(app_types / app_time) ── 所有模块

Remote ────────────────┐
Attitude ──────────────┤
Power ─────────────────┤
Motor(registry) ───────┼──→ Protection ──→ Motor(output) ──→ 适配层(未实现)
                       │          │                │
                       │          └────────────────┘（output 读保护许可）
                       └──→ Task(编排)

Chassis ──→ Motor(output) 请求
Lift ─────→ Motor(output) 请求
Vehicle ──→ Arm / Gimbal / Shooting ──→ Motor(output) 请求
```

- `Protection` 读取 `Remote`、`Motor(registry)`、`Power`、`Config` 的状态；`Motor(output)` 读取 `Protection` 的输出许可。两者是运行期只读调用，不存在初始化先后环：`Task` 先初始化全部模块，再进入周期评估。
- `Task` 是唯一编排者；模块之间不直接调用对方 update，也不共享可写全局量。

## 4. 启动顺序

由 `Core/Src/freertos.c` 的 `MX_FREERTOS_Init()` 在 `USER CODE BEGIN RTOS_THREADS` 区创建唯一新增线程：

```c
g_app_control_task_handle = osThreadNew(app_control_task, NULL, &app_control_task_attributes);
if (g_app_control_task_handle == NULL) { Error_Handler(); }
```

原有 `CAN1_Sending_Service`、`CAN2_Sending_Service`、`DJI_Motor_Service` 线程保持不变（`freertos.c` 弱定义，`Bsp/CAN_bsp.cpp` 与 `User_Driver/Motor_Driver/DJI_motor_driver.cpp` 提供强定义；已用 ELF 符号核对最终调用的是强定义）。

线程首次运行时的应用初始化顺序（`Task/app_task.c: app_modules_init`）：

1. `app_config_init` —— 校验 9 台电机与各机构配置，缓存缺项报告
2. `app_motor_init` —— 只注册通信必需项齐全的电机（本轮 0 台）
3. `app_remote_init` / `app_attitude_init` / `app_power_init` —— 默认离线 / 未实现 / 未配置
4. `app_chassis_init` / `app_lift_init`
5. `app_protection_init` —— 汇总各模块状态，初始 `DISABLED`，输出不允许
6. `app_vehicle_init` —— 本车专有模块（Arm / Gimbal+Shooting）

初始化不等待硬件、不创建额外 RTOS 对象；线程随后以 `APP_CONTROL_PERIOD_MS = 2 ms` 进入周期循环（`osDelayUntil`）。2 ms 是首轮取值，未做实时性验证。

## 5. 周期数据流

每周期固定顺序（`Task/app_task.c: app_control_step`）：

```text
遥控 update ─ 电机反馈 update ─ 姿态 update ─ 功率 update        （一致快照）
        ↓
遥控复位开关上升沿 → 只清锁存故障
        ↓
保护：set_arm_request / set_mode_active → evaluate → 唯一 output_allowed
        ↓
app_motor_clear_requests()                                       （旧请求必然失效）
        ↓
按模式生成本周期请求（底盘；抬升映射待确认，当前不生成）
        ↓
chassis_update → lift_update → vehicle_update(Arm 或 Gimbal/Shooting)
        ↓
app_motor_submit()                                               （唯一输出提交入口）
        ↓
output_allowed 下降沿 → motor_disable_all + lift_stop + vehicle_stop
        ↓
周期计数 / 超时统计 / 报告
```

关键约束：

- 每周期只有 `app_motor_submit()` 一个函数写最终电机命令；各模块只能写 staging 请求。
- `app_motor_clear_requests()` 在周期开头执行，模式切换、失联、故障都不会残留旧请求。
- 中断内不做运动学、不提交输出；CAN 中断只做既有驱动自己的收发。
- 输出许可为假时，最终命令一律 `enabled=false`，且不保留上一周期命令。

## 6. 关键接口

### 6.1 公共模块

| 模块 | 关键接口 | 说明 |
| --- | --- | --- |
| Common | `app_time_now_ms()`、`app_time_is_fresh(now,stamp,timeout_ms)`、`app_clampf()` | 时间 ms、限幅；`APP_TIME_NEVER` 表示“从未更新” |
| Config | `app_config_init()`、`app_config_status()`、`app_config_motor(slot)`、`app_config_motor_missing(slot)`、`app_config_motor_comms_usable(slot)`、`app_config_motor_motion_usable(slot)`、`app_config_find_motor(role)` | 只读配置表 + 缺项位标志；9 台电机 |
| Remote | `app_remote_update(now)`、`app_remote_get_snapshot(out)`、`app_remote_ingest(raw)`、`app_remote_status()` | 快照：`forward/turn/aux1/aux2` 归一化、`mode`、`online`；默认离线 |
| Chassis | `app_chassis_update(request,now)`、`app_chassis_get_state()`、`app_chassis_stop()` | 请求 m/s、rad/s；输出左右履带 rad/s |
| Lift | `app_lift_update(request,now)`、`app_lift_get_state()`、`app_lift_stop()` | 前后机构目标 rad；同步/行程待标定 |
| Attitude | `app_attitude_update(now)`、`app_attitude_get_snapshot()`、`app_attitude_ingest(sample)` | 默认 `NOT_IMPLEMENTED`，`valid=false` |
| Power | `app_power_update(now)`、`app_power_get_budget()`、`app_power_limit_scale(req,scale)` | 预算未知时 `scale=0` |
| Protection | `app_protection_evaluate(now)`、`app_protection_output_allowed()`、`app_protection_raise(flags)`、`app_protection_clear_latched()`、`app_protection_report()` | 唯一输出许可；故障锁存需显式清除 |
| Motor | `app_motor_init(now)`、`app_motor_update_feedback(now)`、`app_motor_feedback_get()`、`app_motor_clear_requests()`、`app_motor_request(slot,mode,value)`、`app_motor_submit(now)`、`app_motor_output_get()`、`app_motor_disable_all(now)` | 注册门控、请求 staging、唯一提交、单位换算 |
| Task | `app_control_task()`、`app_control_task_attributes`、`app_task_get_report()` | 唯一周期任务与报告 |

### 6.2 工程车专有

| 模块 | 关键接口 | 说明 |
| --- | --- | --- |
| Arm | `app_arm_init/update/get_state`、`app_arm_set_joint_target(joint,rad)`、`app_arm_get_joint_target()`、`app_arm_clear_joint_targets()`、`app_arm_get_joint_feedback()`、`app_arm_set_end_target()`、`app_arm_clear_end_target()`、`app_arm_stop()` | 五关节：Yaw / 大臂 Pitch / 小臂 Pitch / 爪子 Roll / 爪子开合 |
| Arm 运动学 | `app_arm_kinematics_forward(joint_rad,out_pose)`、`app_arm_kinematics_inverse(pose,joint_rad)`、`app_arm_kinematics_status()` | 首轮一律 `NOT_IMPLEMENTED`，不输出伪结果 |
| Arm 任务 | `app_arm_task_request(task)`、`app_arm_task_cancel()`、`app_arm_task_get_state()` | 合法请求置 `UNSUPPORTED`，永不 `COMPLETED` |
| Vehicle | `app_vehicle_update(remote,output_allowed,now)`、`app_vehicle_stop()`、`app_vehicle_get_report()` | 通道映射未确认，不猜关节目标 |

### 6.3 炮台车专有

| 模块 | 关键接口 | 说明 |
| --- | --- | --- |
| Gimbal | `app_gimbal_init/update/get_state`、`app_gimbal_set_target(axis,rad)`、`app_gimbal_get_target()`、`app_gimbal_clear_targets()`、`app_gimbal_get_feedback()`、`app_gimbal_stop()` | Yaw 6020 / Pitch 6006（达妙，不得用 DJI 驱动） |
| Shooting | `app_shooting_update(request,now)`、`app_shooting_get_state()`、`app_shooting_stop()` | 摩擦轮 rad/s；拨弹转速未标定，不写占位请求；卡弹恒 `UNKNOWN` |
| Vehicle | `app_vehicle_update/stop/get_report` | MANUAL 只更新云台，AUTO 时发射请求显式无效 |

## 7. 单位与状态约定

- 应用层：角度 `rad`、角速度 `rad/s`、长度 `m`、时间 `ms`（函数名带单位后缀）。归一化量 `[-1,1]` 只表示比例，不表示物理安培/伏特。
- 驱动侧：DJI 设定的归一化量、反馈的转子 rpm、转子圈数，只在 `App/Motor/app_unit.c` 换算；方向、减速比、零位全部来自本车配置，任一缺失返回 `UNCONFIGURED`。
- 状态码：`APP_STATUS_UNCONFIGURED / NOT_READY / OK / OFFLINE / NOT_IMPLEMENTED / INVALID_PARAM / FAULT`。未实现接口显式返回 `NOT_IMPLEMENTED`；反馈快照带 `valid` 与 `timestamp_ms`，未接入时 `valid=false`。
- 配置有效性：每台电机有 `configured` 与 9 个独立 `*_valid`；机构参数同样有独立标记。占位值（0、1.0f 等）不会自动生效。
- 运行状态：`DISABLED / WAIT_CONFIG / WAIT_INPUT / WAIT_DEVICE / ARMED / RUNNING / FAULT`；首轮上电为 `WAIT_CONFIG`。

## 8. C/C++ 边界与构建系统

- 应用层全部是 C（`App/`、`Config/`、`Task/`），头文件带 `extern "C"` 保护，不暴露 C++ 类。既有 C++ 驱动与 `CAN_bsp` 不参与应用层 include，避免 C++ 语法泄漏到 C 入口。
- `Task/app_task.h` 是 freertos.c 唯一新增 include；线程创建写在 `USER CODE BEGIN RTOS_THREADS` 区，CubeMX 重新生成不会丢失。
- `CMakeLists.txt` 改为显式收集 `Bsp/User_Driver/Task/App/Config` 下的 `.c/.cpp`（不再用 `*.*`，避免把 README 当源文件），加入 `App`、`Config` include 路径；MCU、链接脚本、启动文件、HAL、FreeRTOS 与 C++ 工具链未改。
- 两车 `CMakeLists.txt` 内容一致；源文件集合由本车实际目录决定。

## 9. 构建结果与警告

环境：本机 STM32CubeCLT（GNU Arm GCC/G++ 14.3.1、CMake 4.3.1、Ninja 1.13.2）。

```powershell
# 工程车
cd "project file/Engineering robots"
cmake --preset Debug -B "../../work/framework-engineer"      # exit 0
cmake --build "../../work/framework-engineer" --clean-first -j 4   # exit 0

# 炮台车
cd "project file/Turret Robot"
cmake --preset Debug -B "../../work/framework-turret"        # exit 0
cmake --build "../../work/framework-turret" --clean-first -j 4     # exit 0
```

- 两车 ELF 同名 `dx_assignment5_freertos.elf`，分别位于 `work/framework-engineer/` 与 `work/framework-turret/`；烧录前必须按目录确认来源。
- 内存：工程车 RAM 57696 B（44.02%）、FLASH 38164 B（3.64%）；炮台车 RAM 57656 B（43.99%）、FLASH 38532 B（3.67%）。基线为 RAM 56736 B、FLASH 27724 B。
- 告警：两车完整重编译日志各 19 行 `warning:`，全部来自既有驱动：`Bsp/CAN_bsp.*`、`User_Driver/Motor_Driver/*`、`User_Driver/pid_driver.*`，类型为 `-Wreorder`（成员初始化顺序）与 `-Winvalid-offsetof`（对非标准布局类型使用 `offsetof`）。**新增 App/Config/Task 源文件 0 条告警**；因本轮未修改这些驱动，既有告警未消除，也不能据此认为驱动无问题。

## 10. 公共模块同步与两车差异

公共文件由工程车目录复制到炮台车，复制后逐文件 `diff` 为空：

```text
App/Common、App/Remote、App/Chassis、App/Lift、App/Attitude、App/Power、
App/Protection、App/Motor、App/Vehicle/app_vehicle.h、
Task/app_task.c、Task/README.md、Config/app_config.h
```

两车只有以下差异（预期差异，不应被“同步”覆盖）：

| 差异 | 工程车 | 炮台车 |
| --- | --- | --- |
| `Config/app_config.c` | `ENGINEER`；9 电机 = 2×3508 履带 + 2×6020 抬升 + 2×4310 + 2×3508 + 1×2006 机械臂 | `TURRET`；2×3508 履带 + 2×6020 抬升 + 6020 Yaw + 达妙 6006 Pitch + 2×3508 摩擦轮 + 2006 拨弹 |
| 专有模块 | `App/Arm/` | `App/Gimbal/`、`App/Shooting/` |
| 接入层实现 | `App/Vehicle/app_vehicle.c` 调机械臂 | `App/Vehicle/app_vehicle.c` 调云台/发射 |
| 车辆 README | 机械臂说明 | 云台/发射说明 |

修改公共模块时必须同时改两车并重新 diff；硬件映射与专有控制差异只允许出现在上表位置。

## 11. 已实现 / 仅骨架 / 未实现

已实现（代码路径存在，未上板）：

- 配置表与独立有效性标记、缺项位标志报告、`UNCONFIGURED / NOT_READY / OK` 判定。
- 电机注册门控（未确认不注册）、请求 staging、唯一输出提交、旧命令失效、最终命令单一读者。
- 机构单位与转子单位换算及其配置门控。
- 整车周期任务（2 ms）、启动顺序、报告与周期超时统计。
- 保护状态机：故障锁存、输出许可、失联/未配置/未就绪分级。
- 遥控快照与逻辑模式枚举（无接收硬件，默认离线）。
- 底盘差速换算式、抬升位置请求、机械臂单关节目标与限位门控、云台单轴目标与限位门控、摩擦轮成对请求门控。

仅骨架（接口/状态齐全，行为占位）：

- 姿态接口（`NOT_IMPLEMENTED`）、功率预算（无来源、无物理模型）。
- 机械臂任务状态机（合法请求只置 `UNSUPPORTED`）。
- 卡弹状态（恒 `UNKNOWN`）、拨弹（不写占位转速请求）。

未实现：

- 遥控 DT7/DR16 的 UART/DMA 接收、通道/开关映射。
- BMI088/EKF 姿态移植。
- 达妙 4310/6006 驱动与协议适配。
- DJI/达妙硬件适配层：构造电机对象、CAN 收发、反馈解析、故障上报。
- 组合帧字节槽与达妙报文的冲突检查。
- 机械臂正解/逆解、抓取/搬运/倾倒动作序列。
- 云台闭环、摩擦轮转速环、拨弹定量与防卡弹（当前只有接口与门控）。
- 功率物理模型、裁判系统数据、支撑/制动策略、遥测上报。

## 12. 待确认硬件与参数（下一轮输入）

1. 9 台电机的 CAN 总线、ID、控制报文/反馈报文 ID、组合帧字节槽；达妙报文是否与 DJI 组合帧冲突。
2. 每台电机控制模式、方向、减速比、零位、限位、PID；`Config/app_config.c` 中逐项打开 `*_valid` 后才能参与输出。
3. 履带轮距、驱动轮半径、最大线速度/转向角速度。
4. 抬升行程、前后同步关系、零位、限位、同步带/齿条参数、载荷与支撑/制动条件。
5. 机械臂坐标系、连杆尺寸、关节零位/限位、末端姿态要求、逆解可达条件、夹持参数。
6. 云台机械限位、Pitch 达妙 6006 的协议与模式、摩擦轮目标转速、拨弹步进量与卡弹判据。
7. DR16 接口/串口参数/DMA、开关与通道映射；IMU 安装方向与用途。
8. 功率来源、可用功率、限幅策略与整车使能条件。

## 13. 验证记录与未验证边界

已执行（本机）：

- 两车 Debug 配置与完整重编译，退出码 0；ELF 与内存占用见第 9 节。
- 新增源文件确实被构建：`work/framework-*/` 下存在 `App/*.obj` 15 个、`Config/app_config.c.obj`、`Task/app_task.c.obj`。
- ELF 符号核对：`app_control_task`、`app_motor_submit`、`app_protection_evaluate`、`app_vehicle_update`、`app_arm_update` / `app_gimbal_update`、`app_shooting_update` 均存在；`MX_FREERTOS_Init` 反汇编含 4 次 `osThreadNew` 调用与 1 次失败停机分支。
- 未实例化会等待反馈的驱动对象：ELF 中不存在 `DJI_motor::DJI_motor` / `can_device_receive::can_device_receive` 构造符号；`.init_array` 与基线完全一致（单个条目），说明没有新增 C++ 全局构造。
- 既有 CAN/DJI 服务强定义存在（`can1_sending_service`、`can2_sending_service`、`DJI_motor_service`），弱定义已被覆盖。
- 公共目录同步 `diff` 为空（第 10 节）。

未执行 / 未验证：

- 未烧录、未连接任何硬件；上电默认无输出、无 CAN 报文属于静态与符号级结论，**没有板上示波器/总线实测**。
- 首轮时本机无可用的主机 C 编译器（无 gcc/clang/MSVC/WSL），纯逻辑配置/模式检查未做主机运行测试；当时只做了代码审查、编译与 ELF 检查。后续已找到 CLion MinGW GCC 15.2.0 并运行主机回归，见 `docs/archive/FRAMEWORK_FIXES.md` 与 `docs/archive/DEEPSEEK_HANDOFF_RESULT.md`。
- 2 ms 周期实时性、栈深度、中断延迟未测量。
- 所有控制参数、方向、零位、限位、PID、连杆尺寸均未标定。

## 14. 已知问题与风险（本轮未修驱动）

以下来自 `docs/archive/BASELINE_REVIEW.md`，本轮通过“不实例化硬件适配”隔离，**不代表已修复**：

1. `DJI_motor` 构造函数 `while (init_offset_f != 1) osDelay(1)` 无限等待：当前没有任何实例，因此不会触发；接硬件前必须改为非阻塞初始化 + 超时。
2. `CAN_bsp.cpp` 接收回调先 `memcpy` 再判空、且不校验 `FilterMatchIndex`/DLC：当前未注册接收设备、未配置过滤器，理论上不会进入该回调；接第一个设备前必须修。
3. CAN 发送队列只保存 `buf_data` 指针：短生命周期载荷可能失效；当前不发送。
4. 发送失败/队列满未检查，信号量可能不归还：当前不发送，但接入后必须先修。
5. CAN2 过滤器组与 `FilterMatchIndex` 映射规则未核验：不得凭假设改偏移。
6. 组合帧旧缓存不会自动失效：已由应用层每周期清请求 + 唯一提交缓解，但驱动层缓存仍需在接入时处理。
7. 姿态参考工程不能整套覆盖：本轮只定义接口，未移植。

其他本轮风险：

- 功率限制只覆盖归一化电流/电压代理量；速度/位置命令的功率约束未实现，目前依赖保护门控。
- 遥控归一化按 DT7 常见 1000..2000 us / 中位 1500 书写常量，`mapping_valid=false` 时不会生效，仍需实车核验。
- `app_control_task` 优先级 `osPriorityAboveNormal`，低于既有 CAN/DJI 服务（`osPriorityHigh`）；若后续周期缩短需重新评估。

## 15. 下一轮调试顺序

1. **通信核对**：确认 CAN 拓扑、9 台电机 ID、组合帧字节槽、达妙报文冲突；填写 `Config/app_config.c` 并逐项打开 `*_valid`。
2. **驱动最小整改**：先修 `DJI_motor` 非阻塞初始化、CAN 回调判空/索引/DLC、队列载荷所有权、发送失败与信号量；两车同步。
3. **单设备接入**：只接 1 台电机，低幅度、限流验证方向与反馈；确认 `app_motor_registered_count()`、`app_motor_feedback_get()` 状态正确。
4. **零位/限位/减速比标定**：打开 `motion_usable`，先底盘与抬升独立验收，再做机械臂单关节 / 云台单轴。
5. **遥控与保护**：接 DR16，确认失联判定、使能开关、模式切换与旧请求失效；确认故障锁存与复位流程。
6. **机构功能**：机械臂正解验证 → 逆解 → 抓取/搬运/倾倒；炮台摩擦轮 → 拨弹 → 防卡弹；再联调发射条件。
7. **功率与整车**：接入功率来源，替换代理量限幅；确认支撑/制动策略后再开放抬升与机械臂故障后恢复。
8. **两车协同**：按根 README 不假设车间通信，补给配合需单独定义通信方式后再验证。
