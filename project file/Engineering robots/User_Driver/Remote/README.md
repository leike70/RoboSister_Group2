# User_Driver/Remote：DR16/DT7 DBUS 协议解析

## 职责

把 `Bsp/remote_uart` 收到的 18 字节 DBUS 原始帧解析成协议字段（四通道、S1/S2
开关、鼠标、键盘），再按 `Config/app_config.c` 的映射参数转换成应用层
`app_remote_input_t`（归一化轴、三档模式、使能/复位请求）。

本目录不访问 HAL、不创建 RTOS 对象、不发电机命令；两车文件完全相同。

## 接口与单位

| 接口 | 说明 |
| --- | --- |
| `dbus_decode(frame, size, sample_ms, &frame_out)` | 校验长度=18、通道∈[364,1684]、开关∈{1,2,3}、鼠标键∈{0,1} |
| `dbus_to_input(&frame, &config, &input)` | 按配置映射轴/死区/档位/使能；配置未确认返回 `UNCONFIGURED` |

- 通道：`ch0..ch3`，中位 1024、范围 364..1684，归一化 `(v-1024)/660` 后限幅 `[-1,1]`。
- 开关：S1/S2 取值 1=上、2=下、3=中（协议 V1.4 附录位域）。
- 协议默认：USART3 100 kbps、8 数据位、偶校验、1 停止位，18 字节/帧。
- 默认映射（`Config` 中可改）：前进 ch3、左转 -ch2、功能轴1 -ch0、功能轴2 ch1；
  S1=档位、S2=使能/复位。

## 调用顺序

`Bsp/remote_uart` 在任务上下文调用 `dbus_decode` → `dbus_to_input` →
`app_remote_publish_frame`（同时发布逻辑输入与原始字段快照）；解析失败时调用
`app_remote_invalidate` 撤销输入并清零原始快照。各车上下档入口可通过
`app_remote_get_aux_snapshot()` 读取四通道、S1/S2、鼠标、键盘。

## 配置项

`Config/app_config.h` 的 `app_remote_config_t`：通道下标、方向符号、死区、
档位/使能开关选择、失联门限。逐项 `*_valid` 为假时本模块拒绝映射。

## 故障与默认行为

- 长度错误、通道越界、开关值非法、鼠标键>1：`INVALID_PARAM`，不发布。
- 映射配置未确认：`UNCONFIGURED`，不发布。
- 不做速率限制、不缓存历史帧、不推断旧工程的重连使能序列。

## 调试步骤

1. 断点看 `dbus_decode` 输出的 ch0..3 与 S1/S2，摇杆/开关逐个动作核对。
2. 核对方括号内通道下标与方向符号是否与实际机构一致后再改 `Config`。
3. 若帧始终非法，先查波特率/校验/电平（DBUS 需板上 DBUS 接口反相链路）。

## 负责人

待认领（公共支持：遥控）。

## 未实现项

- 鼠标/键盘字段只解析、不映射动作（留给各车自行决定）。
- 帧周期、实际电平与通道方向未上板核验。
