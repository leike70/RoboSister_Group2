# Bsp 板级适配

## 职责

- `CAN_bsp.*`：CAN1/CAN2 收发、设备注册、过滤器、发送队列、完成/中止回调、总线错误统计和 bus-off 恢复。
- `remote_uart.*`：USART3 + DMA/IDLE 接收 DBUS 帧，ISR 只保存帧/时间，解析在任务中完成。
- `imu_bsp.*`：SPI1 BMI088 访问；`imu_heater.*`：PF6/TIM10_CH1 加热 PWM 与看门狗。
- 不实现整车策略；不直接调用机构模块。

## CAN 初始化与调用顺序

1. `MX_FREERTOS_Init()` 创建 CAN 队列和信号量。
2. `App_Control` 的 `app_modules_init()` 在电机注册后调用 `can_bsp_start()`。
3. 发送任务只从队列取**独立载荷副本**；`can_bsp_flush_tx()` 在失联/停用/故障/模式退出时撤销旧报文。
4. `can_bsp_service(now)` 在任务上下文做有限频率 bus-off 恢复；ISR 不跑复杂业务。
5. 读取 `can_bsp_get_stats()` 看启动失败、发送失败、队列丢弃、错误和 bus-off 计数。

## 边界

`HAL_CAN_*` 返回值均检查；接收不使用 `FilterMatchIndex` 猜注册槽位，而是校验实例和标准 ID 后查表。实际 CAN 电平、终端电阻、报文周期和电机反馈仍需上板验证。

## 0x31 功率/能量接收

App_Control 在 CAN 启动前调用 `can_bsp_register_std(source_bus, 0x31)` 和 `can_bsp_add_std_observer(app_power_on_can_frame)`。0x31 与电机反馈共用同一接收分派；observer 只在 ISR 保存原始帧/到达时间，解析和策略在 Power 任务上下文完成。来源总线未知时不注册，不由先收到的总线自动选择。
