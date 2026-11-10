# 炮台车上下档扩展入口

本车功能集成人员（T0）维护 `App/Vehicle/app_vehicle.c`，在 `app_vehicle_on_up()` 与 `app_vehicle_on_down()` 中接入各模块。多人开发时，T1修改 `App/Gimbal/`、T2修改 `App/Shooting/`；不要所有人都改这个入口文件。接口和文件分工见 [团队开发说明](../../../../docs/TEAM_DEVELOPMENT.md)。
两个函数分别属于上档与下档，含义自行填写，不预设手动/自动。当前均返回 NOT_IMPLEMENTED，无动作。

中档由公共 Task/Chassis 处理；本车 Vehicle 在中档停止并清除专有目标，不接管底盘公共逻辑。
公共任务通过 `app_vehicle_update()` 完成在线/保护判断与分派。
上/下档互切或退出、失联、停用都会先清旧目标与任务，再由新分支生成当期请求。
使能、协议和物理按键映射不在这里硬编码。

本车已有 Gimbal、Shooting 模块可按需接入；原有反馈与动作占位、标定门控保留。
各机构通过 Motor 请求接口输出，不直接发 CAN，不改另一车文件。
上下档入口需要基础输入（四通道、S1/S2、鼠标、键盘）时，调用
`app_remote_get_aux_snapshot()` 读取带有效性与到达时间戳的快照；失联/非法帧时
`valid=false` 且字段清零，不要自行缓存按下状态。本层不绑定任何机构动作。
后续实现需说明输入单位、参数、故障处理和验证状态。负责人待认领。

三档和遥控技术现状见根目录 `docs/PERIPHERALS_AND_TASKS.md`；多人认领、合并和验收见 `docs/TEAM_DEVELOPMENT.md`。


