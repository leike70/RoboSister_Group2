# Core 工程核心

## 组成

- CubeMX 生成的时钟、GPIO、CAN、USART、SPI、DMA、TIM 初始化。
- `FreeRTOSConfig.h` 与 `freertos.c`：创建 CAN1/CAN2 发送、DJI 服务、`App_Control`、`IMU_Acquire` 任务及 CAN 队列/信号量。
- `main.c`：外设初始化后启动调度器；CAN 不在调度器前启动，由 `App_Control` 在队列/信号量就绪后调用 `can_bsp_start()`。
- `Error_Handler()`：致命错误先关闭 IMU 加热，再进入调试死循环。

## 修改约定

- 优先改 `USER CODE` 区；不要手工覆盖 CubeMX 生成区。
- 新增任务/队列/信号量应同步 `.ioc`、`freertos.c` 和文档。
- 不把中断里的复杂逻辑搬进 `main.c`；共享数据走任务/队列。
- 目前未烧录，时钟、栈、中断延迟和任务周期均未实测。
