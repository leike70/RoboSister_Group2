# RoboSister Group 2

工程车与炮台车是两个独立的 STM32F407 工程，每车一块 C 板；公共模块在两车工程内各保存一份并同步维护，不建立跨工程依赖。工程车负责抓取、搬运和补给，炮台车负责云台与发射。

## AI / 自动化修改前必读

任何 AI 或自动化工具在改代码前，必须依次阅读：本 README → `docs/PROJECT_REQUIREMENTS.md` → `docs/REFERENCE_LIBRARY.md` → 对应车辆 `README.md` → 涉及模块的 `README.md` 与源码。涉及规则、器件、协议或机构时，按资料库索引打开对应原始文件；不得用猜测代替未确认的硬件/协议参数。修改后必须更新 `docs/CHANGELOG.md`，并说明公共同步和验证结果。

## 目录

| 目录 | 用途 |
| --- | --- |
| `project file/Engineering robots/` | 工程车独立工程：应用、板级适配、驱动、配置、任务、CubeMX/HAL/FreeRTOS 与构建文件 |
| `project file/Turret Robot/` | 炮台车独立工程，结构与工程车相同，专有模块不同 |
| `docs/` | 当前需求、多人分工、外设/任务、上板和规则说明 |
| `docs/archive/` | 历史检查、旧提示词和阶段交付记录，仅作留存 |
| `tests/` | 主机回归测试（真实应用/驱动源码 + HAL 桩）与运行说明 |
| `资料文件/` | 比赛规则、电机/遥控/IMU 手册和参考工程，只读参考，不参与构建 |
| `work/` | 运行测试时按需生成的本机缓存；**不随共享目录分发，也不纳入版本管理** |

## 当前状态

- 两车各一块 C 板、两个独立工程；公共代码分别保存在两车工程中，由负责人同步维护。
- 已配置 5 个 RTOS 任务（CAN1 发送、CAN2 发送、DJI 服务、2 ms `App_Control`、2 ms `IMU_Acquire`），以及 UART、CAN、SPI。
- 基础遥控接收/解析/失联判断、三档分派和 BMI088 + 四元数 EKF 姿态解算已接入；PF6/TIM10_CH1 加热 PWM 与温度闭环、预热/校准状态流程也已接入。
- 0x31 功率/能量接收、快照、超时和底盘策略已接入代码；来源总线/超时/上板核验仍未配置，底盘最终电流/力矩适配层未接。
- 中档走公共底盘；上档/下档分别进入各车 `app_vehicle_on_up()` / `app_vehicle_on_down()`，**目前仍是空入口**。
- 电机硬件适配、CAN 总线/电机 ID、机构参数和具体动作仍待完成。
- **尚未上板验证**；PF6 加热 PWM 上电默认 0，温度有效后才允许输出；目标温度/PID/保护阈值待上板整定；车体安装方向未标定，IMU 轴向/精度/时序/任务栈未验证。

## 从这里开始

| 你要做什么 | 入口 |
| --- | --- |
| 工程车开发/构建 | [工程车 README](project%20file/Engineering%20robots/README.md) |
| 炮台车开发/构建 | [炮台车 README](project%20file/Turret%20Robot/README.md) |
| 认领工作包、看文件分工 | [多人开发与任务分工](docs/TEAM_DEVELOPMENT.md) |
| 看项目约定、硬件和单位 | [项目约定与功能需求](docs/PROJECT_REQUIREMENTS.md) |
| 看硬件拓扑和待确认接线 | [硬件拓扑与待确认表](docs/HARDWARE_TOPOLOGY.md) |
| 看遥控、三档、SPI/IMU 和上板步骤 | [外设与任务说明](docs/PERIPHERALS_AND_TASKS.md) |
| 看比赛规则限制 | [比赛规则要求](docs/RULES_REQUIREMENTS.md) |
| 查原始资料、手册和参考工程 | [资料库索引](docs/REFERENCE_LIBRARY.md) |
| 跑主机回归 | [测试说明](tests/README.md) |
| 找历史记录 | [文档目录](docs/README.md) |

## 环境与构建

固件需要 `arm-none-eabi-gcc` / `arm-none-eabi-g++`（GNU Arm Toolchain，已在 14.3.1 上验证）加入 `PATH`，并安装 CMake ≥ 3.22、Ninja。CMake presets 使用工程内的 `cmake/gcc-arm-none-eabi.cmake`，不写死本机绝对路径。

```powershell
cd "project file/Engineering robots"
cmake --preset Debug -B build/Debug
cmake --build build/Debug -j 4

cd "../Turret Robot"
cmake --preset Debug -B build/Debug
cmake --build build/Debug -j 4
```

两车 ELF 同名 `dx_assignment5_freertos.elf`，分别位于各自 `build/Debug/`；烧录前核对工程目录。最新本机构建结果见两车 README 和 `docs/PERIPHERALS_AND_TASKS.md`。

主机回归需要本机原生 GCC/CMake/Ninja/CTest；若 `gcc` 不在 `PATH`，按 [测试说明](tests/README.md) 用 `-Compiler` 指定。首次运行会自动创建 `work/host-regressions/`，不向硬件收发。

## Git 约定

- 构建目录、ELF/MAP/目标文件、日志、IDE 个人状态和整个 `work/` 已由根 `.gitignore` 排除；不要把本机产物提交。
- 各车 `.gitignore` 继续排除各自构建输出；CMake、CMakePresets、工具链文件、HAL、FreeRTOS、启动文件、链接脚本、`.ioc`、测试和 `资料文件/` 均保留在版本管理中。
- 共享 IDE 配置（如团队约定的 `tasks.json`/`launch.json`）不应按个人缓存处理；个人 workspace 状态才忽略。
- 推送前确认 `git status` 中没有 `work/`、`build/`、`*.elf`、`*.log` 或个人 IDE 文件。
