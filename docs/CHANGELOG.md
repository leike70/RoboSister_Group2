# CHANGELOG

## 2026-11-10 — CubeMX 任务登记补齐

- 两车 `.ioc` 的 FreeRTOS Tasks 增加 `App_Control`（AboveNormal，512 words）和 `IMU_Acquire`（Normal，2048 words），与此前源码中的 2 ms 任务一致。
- 两车 `freertos.c` 改为按 `.ioc` 对应的属性各创建一次任务；用户代码区只保留句柄别名和创建失败检查。任务入口、周期和业务逻辑不变。
- Task 文档与 `.ioc` 均登记五个任务。尚未实际打开 CubeMX 或重新生成代码；打开项目后需核对任务表，再生成一次代码并检查差异。当前已核对 `.ioc` 与源码并执行编译/测试。
- 两车干净构建均通过：工程车 RAM 62536 B / FLASH 94040 B，炮台车 RAM 62496 B / FLASH 93884 B；主机回归 12/12 通过，公共文件同步检查通过。

## 2026-11-10 — 共享目录清理

- 移除本机 `work/` 中的构建、日志、阶段复核和旧链路恢复副本；该目录被 `.gitignore` 排除，运行测试时会重新生成。
- 清除 IMU 参考工程的 Keil/Make 编译输出，保留参考源码、工程配置、说明书与两车原始资料。
- 根 README、两车 README、测试说明与当前外设文档改为指向可复现的构建命令，不再要求接手者依赖作者本机缓存。两车固件源码未改。

## 2026-11-10 — 评审问题修复（发送阻塞/ISR/适配/DLC/快照）

- CAN 发送任务改为先阻塞取队列消息，再等待邮箱信号量；空队列不再空转。增加 TX generation，清队列后已出队但未发送的旧命令会失效；`can_bsp_stop_services()` 供测试/停止路径退出阻塞任务。
- CAN 接收/发送完成/取消/错误回调全部改用 `taskENTER_CRITICAL_FROM_ISR()`/`taskEXIT_CRITICAL_FROM_ISR()`；测试桩在模拟中断上下文检测误用任务临界区。
- CAN 错误中断启用 `CAN_IT_ERROR`，补 CAN1/CAN2 SCE NVIC 与 `CANx_SCE_IRQHandler`；`can_bsp_start()` 支持部分启动重试，`can_bsp_service()` 在未启动时有限重试、bus-off 后按 500 ms 周期恢复；启动/恢复失败有统计。
- `app_motor` 增加真实适配层握手 `app_motor_set_adapter_ready()`；未就绪时 `app_motor_submit()` 返回 `NOT_IMPLEMENTED`，所有最终命令 `enabled=false`，不再把速度命令标成可执行。`current_limit_norm` 保留给适配层消费。
- 旧归一化预算与 0x31 底盘源解耦：非底盘只受 `app_power_limit_scale()` 的旧预算约束，不再因底盘源未配置/离线变化；新增底盘/非底盘隔离测试。
- DJI 回调校验 `rx_dlc==8`；DLC=0/短帧不更新编码器、消息计数、时间戳或就绪状态；测试覆盖短帧后正常帧。
- Power 快照改为“局部变量计算 + 一次短临界区整体发布”：`app_power_update()` 不在中断关闭期间解析/算策略；`get_feedback()/get_policy()/get_budget()` 只在短临界区复制，不再自旋。ISR 原始帧仍用独立短临界区保存。
- 最终构建：工程车 RAM 62528 B / FLASH 94020 B，炮台车 RAM 62488 B / FLASH 93864 B，exit 0；告警各 16 行（既有驱动 reorder/offsetof）。
- 主机回归 12/12 通过；新增空队列阻塞、ISR 临界区误用、部分启动/bus-off、DJI 短帧、适配层未就绪、底盘/非底盘隔离覆盖。
- 未上板：0x31 实际总线/周期/超时、SCE 波形、真实电流/力矩限幅、断电恢复均未实测；生产 `hardware_verified=false`，适配层未接，不能宣称实车功率限制完成。

## 2026-11-10 — 资料库导航

- 新增 `docs/REFERENCE_LIBRARY.md`，按规则、遥控、大疆/达妙电机、IMU 和期中机械设计整理 `资料文件/` 的原始资料入口、对应代码/文档及核验边界。
- 根 README 将资料库索引加入 agent 修改前必读顺序；`docs/README.md` 增加导航。仅修改文档，未改变两车源码或构建。

## 2026-11-10 — 0x31 功率/能量反馈与底盘策略

### 协议与配置

- 按 `docs/RULES_REQUIREMENTS.md` 实现标准 CAN 0x31、DLC=4、小端 int16：底盘功率 0.1 W、缓冲能量 0.1 J；负值保留。
- `Config/app_config.h` 的 `power` 段新增：来源总线及其有效标记、超时及其有效标记、硬件核验标记、底盘功率/缓冲能量规则限制、低能量阈值及独立标记。
- 两车规则值：工程车 50 W / 500 J，炮台车 40 W / 400 J；规则版本仍有疑点，最终以赛务为准。
- 来源总线仍为 `APP_CAN_BUS_NONE`、`source_bus_valid=false`、`source_timeout_valid=false`、`hardware_verified=false`；没有猜 CAN1/CAN2，也没有打开验证标记。

### CAN 接收

- `Bsp/can_bsp_api.h` 新增 `can_bsp_register_std()`、`can_bsp_add_std_observer()`；沿用上一轮接收分派，没有第二套抢占回调。
- 0x31 接收器使用已校验 CAN 实例/ID/DLC；observer 在 ISR 只保存总线、ID、原始字节、DLC、到达时间。
- 设备注册增加重复 `(CAN 实例, ID)` 检查；观察者共享同一分派。
- `Bsp/CAN_bsp.cpp` 使用 `app_time_now_ms()` 记录到达时间；测试边界提供等价时间桩。

### Power / Motor / Protection

- `App/Power` 新增 `app_power_on_can_frame()`、`app_power_update()`、`app_power_get_feedback()`、`app_power_get_policy()`。
- 快照区分 `measured_chassis_power_w`、`buffer_energy_j`、`allowed_power_w`、旧代理 `max_total_output_norm`；`available_power_w` 仍是旧字段恒 0，不装入实测功率。
- 上电无数据无效；持续有效帧在线；超时撤销可用；非法帧/错误总线不刷新有效到达时间；时间戳不被读取刷新。
- 底盘策略状态：正常、功率受限、低能量、不可用；超功率按 `limit/measured` 保守降额（不是功率模型）；低能量阈值默认规则 0 J；来源未核验/超时禁止底盘运动。
- `app_motor_submit()` 只对 `TRACK_LEFT/TRACK_RIGHT` 施加底盘策略；速度/电流/电压按比例，位置目标不缩放、受限时直接不输出；`app_motor_output_t` 新增 `current_limit_norm`、`power_limited`。
- `app_protection` 不再把底盘功率源状态当整车全局停用；保护层仍报告 `power_status`，底盘由 Motor 策略门控。`WAIT_POWER` 枚举保留但当前不作为全局状态。
- App_Control 在 CAN 启动前按配置总线注册 0x31；功率源失效/低能量/降额时清旧请求和旧 CAN 队列，恢复从当前输入重新生成，不重放旧目标。
- `app_task_report_t` 增加实测功率、缓冲能量、底盘缩放、来源在线、核验状态、错误位、有效帧到达时间。

### 测试

- 新增/扩展 `power_motor_contract_regression.c`：字序、单位、正负值、DLC、错误总线、超时、时间回绕、低能量、超功率、位置不缩放、`current_limit_norm`。
- 新增 `config_power_limits_regression.c`，分别编译工程车/炮台车配置，校验 50/500 与 40/400，并确认来源仍未知。
- `can_dji_regression.cpp` 增加 0x31 共享 observer 路由/到达时间验证。
- 现有 10 组保留；总回归 **12/12 通过**。

### 构建与限制

- 工程车：RAM 62528 B / FLASH 94020 B，exit 0。
- 炮台车：RAM 62488 B / FLASH 93864 B，exit 0。
- 构建告警各 16 行，仍为既有 DJI/pid 驱动的 `-Wreorder`/`-Winvalid-offsetof`，未清理。
- 0x31 解析/策略/最终输出字段已有主机测试；**未上板**，未验证实际总线、报文周期、功率测量、能量变化、驱动电流/力矩限幅和断电恢复。
- 电机硬件适配层仍未接；`current_limit_norm` 是接口字段，不能据此声称实车限功率已完成。

## 2026-11-10 — 上一轮基础通信与保护加固（摘要）

- CAN BSP 修复接收空指针/越界风险，发送队列改独立载荷；增加启动、bus-off、统计和旧报文撤销。
- DJI 构造移除无限等待，增加生命周期/超时/显式输出使能，不自动发送。
- 保护增加逻辑急停、CAN 故障、功率许可；App_Control 增加周期超时判据和 CAN 健康上报。
- 遥控配置区分协议在线、映射参数有效和上板核验。
- 新增 CAN/DJI、真实 Config/Motor/Power/Protection、功率门控合同测试；修复两车公共文件同步和重复 `Core/Src/tim.h`。
