# 框架四项修复与验证

> 历史记录：本文保留对应阶段的事实与任务。当前开发从 [文档目录](../README.md) 开始；接口状态见 [当前遥控与任务说明](../PERIPHERALS_AND_TASKS.md)。旧文提到的根README详细章节已移到 [项目约定](../PROJECT_REQUIREMENTS.md)。

> 当前状态提示：本文写于三档改造前，提到的 `APP_MODE_VEHICLE_MANUAL`/`AUTO` 与旧遥控
> 接收链路已作废；当前模式为 `APP_MODE_CHASSIS` / `APP_MODE_VEHICLE_UP` /
> `APP_MODE_VEHICLE_DOWN`，接口与分工以 `docs/PERIPHERALS_AND_TASKS.md` 为准。
> 本文的四项修复结论、公共同步与测试方法仍然有效。

日期：2026-11-10。根据用户“直接改”的授权，修复复核记录中的四项问题，公共修改已同步两车。

## 已修改的行为

1. **工程车模式隔离**：只有在线、保护允许的 VEHICLE_MANUAL 模式会调用机械臂更新。切到 CHASSIS、DISABLED、UNKNOWN、尚未实现的 AUTO，或失联/无许可/空输入时，清除关节与末端目标、取消任务。重新进入手动模式不会自动恢复旧目标。
2. **故障恢复路径**：新增 `app_protection_resolve(flags)`。监督任务核实原因消失后仅清除指定活动故障，历史锁存仍禁止输出；活动故障全部解除后才可显式 `clear_latched()`。复位清除旧使能/动作标志，下一次重新评估后决定是否运行。保护写接口规定由控制/监督任务串行调用。
3. **一致输入快照**：新增两车各自的 `App/Common/app_critical.h/.c`，保存/恢复 Cortex-M PRIMASK，并使用内存屏障。遥控原始帧、发布快照、状态码及姿态快照整帧读写受到短临界区保护。普通任务/ISR 可调用输入入口，NMI/HardFault 不调用；调用方仍需确保源缓冲在一次调用期间稳定，不得直接传入正在被 DMA 改写的缓存。
4. **姿态有效性**：`valid=false` 输入发布无效快照并返回 NOT_READY；有效标志为真但含 NaN/Inf 时发布无效快照并返回 INVALID_PARAM。无效快照的数值清零、时间戳保留，避免继续使用旧的有效姿态。NULL 只拒绝调用。周期 update 不刷新旧时间戳，也不恢复无效状态。

本轮未实现 AUTO 动作、遥控硬件映射或 IMU 驱动；没有填写未知电机参数，没有修改底层电机/CAN 驱动。

## 实际运行的回归测试

新增 `tests/`，使用本机已安装的 CLion MinGW GCC 15.2.0 编译 Windows 测试程序；未安装或下载编译器。先前仅在 PATH 中找不到主机编译器，后续检查安装目录找到了可用工具。

运行入口：

```powershell
.\tests\run-regressions.ps1
```

测试目录为 `work/host-regressions/`。最终结果：**3/3 测试组通过，0 失败**，测试源以 `-Wall -Wextra -Werror` 编译。

| 测试组 | 实际覆盖 |
| --- | --- |
| engineer_regressions | 使用工程车真实 Remote/Attitude/Protection/Arm/Vehicle 源码，验证故障置位、部分解除、锁存及复位；遥控超时；无效/NaN/Inf 姿态；7 种退出或拒绝条件下旧关节目标与任务清除、重新进入手动不恢复目标 |
| turret_regressions | 使用炮台车真实公共模块源码，验证同样的故障恢复、遥控超时与姿态有效性逻辑 |
| critical_mask_regressions | 使用实际生产 `app_critical.c`，以 CMSIS 寄存器桩验证嵌套、原先已屏蔽中断时仍保持屏蔽、最终恢复原掩码及内存屏障调用 |

前两个测试组各运行一轮三线程数据竞争场景：生产者发布 40000 帧，控制线程更新遥控并读取姿态，第三线程读取发布快照；检查字段与时间戳都属于同一完整帧。使用 Windows 临界区模拟单核互斥，不能代替板上的中断延迟测量。

测试用显式配置、时间与电机请求记录桩隔离硬件，行为逻辑调用实际应用源码。测试中的有效配置仅存在测试程序中，车辆 `Config/app_config.c` 的默认配置未被打开。

## 两车固件编译

在项目根目录分别执行：

```powershell
cmake --build 'work/review-engineer' -j 4
cmake --build 'work/review-turret' -j 4
```

两车均退出 0，新 `app_critical.c` 已通过自动源文件发现机制纳入固件。

| 工程 | RAM | FLASH | ELF |
| --- | --- | --- | --- |
| 工程车 | 57696 B | 38340 B | `work/review-engineer/dx_assignment5_freertos.elf` |
| 炮台车 | 57656 B | 38680 B | `work/review-turret/dx_assignment5_freertos.elf` |

公共 Common、Remote、Attitude、Protection 目录（含说明文件）逐文件哈希比对，差异为 0。固件编译保留既有驱动的成员初始化顺序和 `offsetof` 警告；本轮新增/修改应用源文件未见新增警告。

## 仍未验证的边界

未烧录或连接实车，尚未测量临界区耗时、中断延迟、2 ms 周期和任务栈。IMU 超时门限、遥控刻度/开关映射、功率闭环限制与原有驱动问题仍按复核记录列为后续工作；主机回归通过不能作为这些硬件功能完成的依据。
