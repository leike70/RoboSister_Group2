# 框架收尾复核结果

> 历史记录：本文保留对应阶段的事实与任务。当前开发从 [文档目录](../README.md) 开始；接口状态见 [当前遥控与任务说明](../PERIPHERALS_AND_TASKS.md)。旧文提到的根README详细章节已移到 [项目约定](../PROJECT_REQUIREMENTS.md)。

> 更新（2026-11-10）：本文第 1–6 节记录“框架收尾”那一轮；随后用户要求补齐真实遥控链路，
> 已按第 7 节实现并经主机回归与两车编译验证。当前遥控接口、映射与验证结果以
> `docs/PERIPHERALS_AND_TASKS.md` 和本文第 7 节为准；第 5 节中“遥控未接入”的条目已被取代。

日期：2026-11-10。范围：在两车现有工程上做一致性复核、过时文档更正与验收，**没有重新搭建框架，没有实现机构动作或新接收协议**。

## 1. 实际修改文件

本轮源码逻辑未发现需要修复的缺陷；改动集中在注释与文档，另有 1 处公共头文件注释更正（两车同步）。

| 文件 | 修改内容 |
| --- | --- |
| `README.md`（项目根） | 更新当前状态；更正阅读入口表（文档历史/当前分工）；RTOS 改为四个任务；遥控器行改为“未接入物理接收、仅逻辑输入接口与通用串口初始化”；删除“上档手动/下档自动、右开关重连使能”描述；待确认项改为新方案待定；构建记录指向最新文档 |
| `project file/Engineering robots/README.md` | 删除“USART3/DR16、DMA/IDLE、DBUS 已完成接入”的过时描述；重写“当前状态”（三档入口、四个任务、最终构建结果、告警、串口现状、主机回归、无主机编译器说法已删）；本车模块说明改为上/下档入口；构建命令与内存更新 |
| `project file/Turret Robot/README.md` | 同上；删除“遥控串口、DMA、DBUS 与模式已接入”一行；专有模块表把 `MANUAL/AUTO` 改为中档/上档/下档分派说明；构建命令与内存更新 |
| `project file/*/Task/README.md` | 周期顺序改为逻辑复位请求；补充上/下档由 `app_vehicle_update` 分派到本车空入口；两车同步 |
| `project file/*/App/Protection/app_protection.h` | 注释更正：`app_protection_set_arm_request` 为“逻辑使能请求（来自抽象输入，物理映射未定）”，不再写“遥控开关”；两车同步 |
| `docs/PERIPHERALS_AND_TASKS.md` | 更新日期与验证结果；补充 ELF 证据（无 DMA/UART 接收路径、旧链路符号为零）；写入本轮实际构建路径、内存、回归与告警结果 |
| `docs/archive/FRAMEWORK.md` | 在既有历史提示上补充：MANUAL/AUTO、`app_remote_ingest`、DR16 等内容已作废，当前以三档接口为准；标注“无主机编译器”为首轮情况，后续已找到 MinGW GCC |
| `docs/archive/FRAMEWORK_FIXES.md` | 增加当前状态提示（三档改造后 MANUAL/AUTO 作废，修复结论与测试方法仍有效） |
| `docs/archive/FRAMEWORK_REVIEW.md` | 在“后续状态”提示中注明遥控链路已撤下，DR16 建议仅作历史记录 |
| `tests/README.md` | 补充执行策略限制时的运行命令与 clean-first 说明；保留原有测试范围说明 |
| `docs/archive/DEEPSEEK_HANDOFF_RESULT.md` | 新增（本文） |

未修改：`App/Remote`、`App/Vehicle`、`Task/app_task.c`、`Config/app_config.c`、`Core/Src`、`.ioc`、CMake、电机/CAN 驱动、`tests/` 测试源码。`work/remote-chain-before-removal/` 未改动、未纳入编译；`D:/ROBOMASTER/final task/` 未动。

## 2. 三档调用关系

| 档位 | 逻辑模式 | 执行位置 |
| --- | --- | --- |
| 中档 | `APP_MODE_CHASSIS` | 公共 `Task/app_task.c`：`app_protection_set_mode_active(true)` 后生成 `app_chassis_request_t`，交 `app_chassis_update()`；本车 `app_vehicle_update()` 看到非上/下档会清空本车旧目标，不接管底盘 |
| 上档 | `APP_MODE_VEHICLE_UP` | 公共任务只做保护与分派，`app_vehicle_update()` → 本车 `app_vehicle_on_up()`（工程车：Arm；炮台车：Gimbal/Shooting）。当前均返回 `NOT_IMPLEMENTED`，无运动请求 |
| 下档 | `APP_MODE_VEHICLE_DOWN` | 同上，进入本车 `app_vehicle_on_down()`，与上档互相独立 |

周期顺序：`app_remote_update` 取一致快照 → 复位请求上升沿清锁存故障 → 保护评估得唯一 `output_allowed` → `app_motor_clear_requests()` → 中档生成底盘请求 → `chassis/lift/vehicle` 更新 → `app_motor_submit()` 唯一提交。上/下档互切、切回中档、失联、停用或保护拒绝输出时，Vehicle 先清该车保存的旧目标/任务，避免串档。

输入只有抽象接口：`app_remote_publish(&app_remote_input_t)` 发布 [-1,1] 轴值、逻辑模式与显式使能/复位请求；无物理接收调用方，`Config` 中 remote 的 `valid/mapping_valid/timeout_valid` 均为 false，默认离线。两车各四个 RTOS 任务：CAN1 发送、CAN2 发送、DJI 服务、2 ms `App_Control`。（**该段为历史状态，已被第 7 节取代**：当前 USART3+DMA/IDLE 接收 DBUS，remote 配置已启用。）

## 3. 公共代码同步情况

- 逐文件 `diff` 一致：`App/Common`（含 `app_critical`）、`App/Remote`、`App/Chassis`、`App/Lift`、`App/Attitude`、`App/Power`、`App/Protection`、`App/Motor`、`App/Vehicle/app_vehicle.h`、`Task/`（含 README）、`Config/app_config.h`。
- 允许并保留的差异：`App/Vehicle/app_vehicle.c`（各车实现不同）、`Config/app_config.c`（9 台电机角色/型号、`vehicle_kind`、专有机构配置不同）、`App/Arm/` 仅工程车、`App/Gimbal/` 与 `App/Shooting/` 仅炮台车、两车顶层 README。
- 未建立跨工程 include、共享库或 Common 目录；`work/remote-chain-before-removal/` 只是恢复副本，未被任何 CMake/测试引用。

## 4. 实际验证结果

### 4.1 主机回归

执行（本机默认执行策略禁止直接运行脚本，用 Bypass 仅对本进程生效）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run-regressions.ps1
```

为保证是当前源码：先 `cmake --build work/host-regressions --clean-first` 完整重编，再运行脚本。
结果：**3/3 通过，0 失败**（`engineer_regressions`、`turret_regressions`、`critical_mask_regressions`），
测试源以 `-Wall -Wextra -Werror` 编译。主机编译器：CLion MinGW GCC 15.2.0。

覆盖：中/上/下档输入与路由、上下档空入口、切档清目标、中档隔离、离线/停用/空输入、
非法模式与越界/NaN 轴值、故障置位/部分解除/锁存/复位、遥控超时、
无效与 NaN/Inf 姿态、并发生产者/控制/读取 40000 帧快照一致性、临界区嵌套与 PRIMASK 恢复。
测试中的有效配置与电机/时间桩只存在于测试程序，未打开车辆 `Config`。

### 4.2 两车固件

> 以下为首轮“框架收尾”时的构建结果；遥控接入后的结果见第 7.3 节。

```powershell
cd "project file/Engineering robots"
cmake --preset Debug -B "../../work/final-engineer"          # exit 0
cmake --build "../../work/final-engineer" --clean-first -j 4  # exit 0
# RAM 57848 B / FLASH 39384 B
# ELF: work/final-engineer/dx_assignment5_freertos.elf

cd "project file/Turret Robot"
cmake --preset Debug -B "../../work/final-turret"            # exit 0
cmake --build "../../work/final-turret" --clean-first -j 4    # exit 0
# RAM 57808 B / FLASH 39228 B
# ELF: work/final-turret/dx_assignment5_freertos.elf
```

- 告警：两车各 19 行，全部来自既有 `Bsp/CAN_bsp.*`、`User_Driver/Motor_Driver/*`、
  `pid_driver.*`（`-Wreorder` / `-Winvalid-offsetof`）；新增应用源文件 0 条。
- 源码纳入：构建日志确认 `usart.c`、`app_critical.c` 及 App/Config/Task 源文件均参与编译；
  `freertos.c` 含 4 个 `osThreadNew`（含创建失败停机检查）。
- ELF 证据：两车均存在 `can1_sending_service`、`can2_sending_service`、`DJI_motor_service`、
  `app_control_task`、`app_vehicle_on_up`、`app_vehicle_on_down`；不存在 `DJI_motor` 构造函数符号；
  不存在 `dbus`、`remote_uart`、`hdma_usart3`、`MX_DMA_Init` 等旧链路符号；
  `USART*_IRQHandler` 与 `DMA1_Stream*_IRQHandler` 只是启动文件弱默认处理，无 `HAL_UART_IRQHandler` /
  `HAL_DMA_Init`；默认不注册电机、不使能输出、不发 CAN。

### 4.3 未执行

未烧录、未连接硬件；未测量 CAN/USART 波形与电平、临界区耗时、中断延迟、2 ms 周期与任务栈。
`run-regressions.ps1` 通过不等于板上的实时行为已验证。

## 5. 仍未实现 / 待确认

- ~~物理遥控接收（协议、通道、开关编码、失联超时、使能规则均未选择）；`app_remote_publish()` 目前没有调用方。~~（已由第 7 节取代：DBUS 接收链路已接入并有生产调用方。）
- 故障恢复接口 `app_protection_resolve()` 已提供并由主机回归覆盖，但当前没有生产故障源或监督任务调用它；`app_control_task` 只处理逻辑复位请求上升沿。
- 上档/下档动作：四个本车空入口均返回 `NOT_IMPLEMENTED`，不产生运动请求。
- 机械臂正逆解与抓取/搬运/倾倒流程；云台闭环、摩擦轮闭环、拨弹定量与防卡弹。
- 电机硬件适配层、达妙 4310/6006 驱动、CAN 报文映射与冲突检查；IMU 采集/解算。
- 功率物理模型与裁判系统数据；机械支撑/制动策略；遥测上报。
- 板级实时性、任务栈与中断延迟验证。
- ~~`.ioc` 中残留 `NVIC.ForceEnableDMAVector=true`（无 DMA 请求、不生成 DMA 代码），两车 `User_Driver/Remote/` 是移除后留下的空目录；两者都不参与编译，也不构成接收路径。~~（第 7 节已补上 DMA 请求与接收文件，该条目作废。）

## 6. 建议下一步

1. 选定物理遥控方案后，只新增接收适配层调用 `app_remote_publish()`，不改三档分派与保护逻辑。
2. 各车负责人只填本车 `app_vehicle_on_up()` / `app_vehicle_on_down()`，并补本车模块 README 与验证记录。
3. 按 `docs/PERIPHERALS_AND_TASKS.md` 与两车 README 的验收顺序推进：先通信与单设备，再标定与机构，最后整车。

## 7. 遥控链路接入（2026-11-10 后续，取代第 5 节相关条目）

按用户最后要求补齐“遥控接收 → 通道/按键/开关解析 → 在线与失联判断 → 三档分派”。
中档仍走公共底盘，上/下档仍是各车空入口；没有恢复机构动作，也没有把上下档预设为手动/自动。

### 7.1 实现与修改文件

两车相同部分（逐文件 diff 为空）：

- `Core/Inc/dma.h`、`Core/Src/dma.c`（新增）：DMA1 时钟 + Stream1 IRQ。
- `Core/Inc/usart.h`、`Core/Src/usart.c`：USART3 改为 100 kbps、8 数据位、偶校验、
  1 停止位（HAL 9B+EVEN），MSP 中初始化 DMA1 Stream1 Channel4（NORMAL、高优先级）
  并 `__HAL_LINKDMA`；USART1 保持 115200/8N1 备用。
- `Core/Src/main.c`：加入 `MX_DMA_Init()`（在 USART 初始化之前）。
- `Core/Src/stm32f4xx_it.c`、`Core/Inc/stm32f4xx_it.h`：新增 `USART3_IRQHandler`、
  `DMA1_Stream1_IRQHandler`。
- `cmake/stm32cubemx/CMakeLists.txt`：把 `Core/Src/dma.c` 纳入构建。
- `dx_assignment5_freertos.ioc`：补 DMA 请求、NVIC、USART3 参数与初始化顺序，供 CubeMX 再生成。
- `Bsp/remote_uart.h/.c`（新增）：DMA/IDLE 接收、帧保存、重启/停收恢复与统计；
  中断只保存一帧，解析/映射/发布在 `App_Control` 任务上下文。
- `User_Driver/Remote/dbus.h/.c`（新增）+ README：DBUS 18 字节位域解析
  （ch0..ch3、S1/S2、鼠标、键盘）与按配置映射为 `app_remote_input_t`。
- `App/Remote/app_remote.h/.c` + README：明确生产调用链；模式枚举与快照接口不变。
- `Task/app_task.c` + README：初始化最后调用 `remote_uart_init()`；每周期先
  `remote_uart_service(now)` 再取快照。
- `Config/app_config.h`：`app_remote_config_t` 增加通道下标/方向、档位/使能开关选择及
  对应 `*_valid` 标记。
- `tests/uart_stub/usart.h`（新增）、`tests/CMakeLists.txt`、`tests/framework_regression.c`、
  `tests/README.md`：真实 DBUS 帧路径与 HAL 回调桩测试。

两车各自修改：`Config/app_config.c` 的 `remote` 段启用（初始映射/42 ms 门限/0.03 死区，
待上板核验）；两车 README 状态与构建数据更新。

### 7.2 三档与开关约定

- 左 S1：上=`APP_MODE_VEHICLE_UP`，中=`APP_MODE_CHASSIS`，下=`APP_MODE_VEHICLE_DOWN`。
- 右 S2：上=请求使能；中=停用；下=停用并产生故障复位请求（简单电平规则，
  不复用旧工程“下档回中后上档”的重连资格序列）。
- 失联：42 ms 无有效帧判 `OFFLINE`；非法帧/UART 错误立即撤销输入。
- 上下档仍分别进入各车 `app_vehicle_on_up()` / `app_vehicle_on_down()`，当前返回
  `NOT_IMPLEMENTED`，不产生运动请求。

### 7.3 实际验证

- 主机回归：`tests/run-regressions.ps1` **3/3 组通过**（0 失败，CLion MinGW GCC 15.2.0，
  `-Wall -Wextra -Werror`）。新增的 DBUS 测试从构造的 18 字节帧经 HAL 回调进入
  `remote_uart_service`，覆盖通道/开关/鼠标/键盘解析、三档模式、S2 使能/复位、
  轴映射/方向/死区、42 ms 失联、非法长度/开关/通道帧撤销、UART 错误、
  启动失败重试、1 s 停收恢复；抽象 `publish` 用例仍保留但不作为遥控接入证据。
- 两车固件完整重编译（`--clean-first`）：配置与构建退出码均 0。
  工程车 RAM 58024 B / FLASH 48556 B；炮台车 RAM 57984 B / FLASH 48400 B。
  新增遥控/应用源文件 0 条告警；既有驱动 19 行告警保留。
- ELF：存在 `remote_uart_init/service`、`dbus_decode`、`dbus_to_input`、
  `app_remote_publish`、`USART3_IRQHandler`、`DMA1_Stream1_IRQHandler`、`hdma_usart3_rx`；
  `remote_uart_service` 反汇编中可见 `dbus_decode → dbus_to_input → app_remote_publish`
  调用，生产路径已接通；仍无 `DJI_motor` 构造符号。

### 7.4 仍未验证 / 未实现

- 未烧录、未接接收机；USART3 电平、DBUS 反相链路、实际帧周期、四轴方向与 S1/S2 字段
  必须上板核验；42 ms 门限与 0.03 死区是初始值。
- 鼠标/键盘只解析、不映射动作；上下档入口仍为空。
- 电机/机构配置仍全部未确认，因此遥控在线不代表车辆可运动；保护仍会阻止输出。
- 旧工程驱动问题（电机无限等待、CAN 接收校验、发送失败等）仍未修复。

## 8. 遥控链路复核修复：到达时间与原始状态快照（2026-11-10 后续）

按 `work/review-remotelink/REVIEW.md` 的两项发现修复，公共实现两车同步。

### 8.1 修复 1：帧到达时间

- 之前 `Bsp/remote_uart.c` 只在中断保存字节/长度，`remote_uart_service` 用任务处理时刻作为
  `sample_ms`，延迟处理的过期帧会重新获得完整在线窗口。
- 现在接收回调用 `app_time_now_ms()` 保存该帧到达时刻，并在同一临界区与帧字节/长度一起
  交接给任务；`remote_uart_service` 用到达时刻计算新鲜度，超过 `link_timeout_ms` 的帧
  直接丢弃并撤销输入，不发布。
- 新增统计 `stale_frames`（过期帧计数），与 `valid_frames`/`invalid_frames` 区分。
- 复核目录的误判复现 `work/review-remotelink/probe.c` 重新编译运行：修复前
  `online=1 sample_ms=3100`，修复后 `online=0 sample_ms=0`、退出码 0。

### 8.2 修复 2：原始状态快照

- `app_remote.h` 新增 `app_remote_aux_snapshot_t`：`valid`、`timestamp_ms`（到达时刻）、
  `channel[4]`、`switch_s1`、`switch_s2`、`mouse_x/y/z`、`mouse_press_left/right`、`keyboard`。
- 新增 `app_remote_publish_frame(input, aux)`：逻辑输入与原始字段在同一临界区原子发布，
  保证同一帧一致；`app_remote_publish(input)` 保留为无原始字段的轻量入口。
- 新增 `app_remote_get_aux_snapshot(&aux)`：各车上下档入口可读取基础状态，未绑定机构动作。
- 失联、超时、非法帧、UART 错误时快照标记 `valid=false`、字段清零、时间戳
  `APP_TIME_NEVER`，不残留按键/开关按下状态。
- `Task/app_task_report_t` 增加 `remote_aux_valid`/`remote_aux_ms`，便于调试器查看；
  `App/Vehicle` README 说明上下档入口如何读取，但入口仍是空实现。

### 8.3 验证

- 主机回归 `tests/run-regressions.ps1` **3/3 通过**（`-Wall -Wextra -Werror`）：新增
  延迟 30 ms 仍在线、延迟 100 ms 丢弃并离线、32 位时间回绕（32 ms 内在线/64 ms 丢弃）、
  `stale_frames` 计数、原始字段从 HAL 回调完整传递、失联/非法帧后清零、
  并发写入/读取原始快照一致性；原有测试继续通过。
- 两车完整重编译退出 0：工程车 RAM 58072 B / FLASH 49064 B；炮台车 RAM 58032 B /
  FLASH 48908 B；新增/修改应用与遥控源文件 0 条告警，既有驱动 19 行告警保留。
- ELF：`app_remote_publish_frame`、`app_remote_get_aux_snapshot` 存在；接收回调调用
  `app_time_now_ms`；服务链路为 `dbus_decode → dbus_to_input → app_remote_publish_frame`。
- 仍未烧录/上板；USART3 电平、帧周期、通道/开关字段与实时性未实测。
