# 文档目录

日常开发阅读下面文档；各车和各模块的README跟随对应源码保存。旧提示词、首轮交付和复核过程归入 [历史记录](archive/README.md)。版本管理范围（`work/`、构建输出和 IDE 个人状态不提交）见 [根 README](../README.md#git-约定)。

## 当前开发文档

| 文档 | 内容 |
| --- | --- |
| [多人开发与任务分工](TEAM_DEVELOPMENT.md) | 工作包认领、文件维护范围、现有接口、合并与验收、任务模板 |
| [遥控与任务说明](PERIPHERALS_AND_TASKS.md) | 接收链路、按键快照、三档入口、SPI/IMU、接线与配置、当前验证结果 |
| [项目约定与功能需求](PROJECT_REQUIREMENTS.md) | 两车硬件、模块边界、单位、启动/停机、参数与验收阶段 |
| [比赛规则要求](RULES_REQUIREMENTS.md) | 功率、尺寸、物资、裁判系统与待确认的规则问题 |
| [硬件拓扑与待确认表](HARDWARE_TOPOLOGY.md) | 两车 C 板、供电、机构、CAN/遥控/IMU/加热和烧录边界 |
| [资料库索引](REFERENCE_LIBRARY.md) | 规则、遥控、电机、IMU 和 CAD 原始资料的位置、用途及核验边界 |
| [变更记录](CHANGELOG.md) | 本轮文件、接口、影响、公共同步和验证结果 |

## 源码与验证入口

- [工程车README](../project%20file/Engineering%20robots/README.md)：本车设备、开发顺序和构建方法。
- [炮台车README](../project%20file/Turret%20Robot/README.md)：本车设备、开发顺序和构建方法。
- [工程车 IMU 模块](../project%20file/Engineering%20robots/User_Driver/IMU/README.md)：BMI088/EKF、PF6 温控、接口、限制与上板项。
- [炮台车 IMU 模块](../project%20file/Turret%20Robot/User_Driver/IMU/README.md)：两车公共实现，内容相同。
- [任务与调度](../project%20file/Engineering%20robots/Task/README.md)：五个 RTOS 任务、App_Control、IMU_Acquire 与加热看门狗。
- [姿态接口](../project%20file/Engineering%20robots/App/Attitude/README.md)：`app_attitude_*` 单位、超时和调试。
- [主机测试README](../tests/README.md)：12 组回归测试范围与运行方式。
- [公共文件对比脚本](../tests/check-common-sync.ps1)：只读检查两车公共文件是否同步。
- 本机运行测试时才会生成 `work/` 缓存；共享目录不包含它，也不需要复制历史构建输出。

## 文档维护

根README只保留项目导航与简短进度；确定需求更新PROJECT_REQUIREMENTS，分工更新TEAM_DEVELOPMENT，遥控/三档及当前验证更新PERIPHERALS_AND_TASKS。模块接口与调试细节写在模块README。

历史报告保留当时事实，不能当作当前接口说明，也不要重新执行已完成的历史提示词。历史文中“根README第几节”的详细约定，现已集中到PROJECT_REQUIREMENTS。
