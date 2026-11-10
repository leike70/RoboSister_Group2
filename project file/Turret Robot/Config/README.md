# 本车配置表（Config/）

## 文件

- `app_config.h`：两车一致的结构、有效性标记与查询接口。
- `app_config.c`：**本工程专用**内容，工程车与炮台车各一份，不得互相复制映射。
- `imu_config.h`：IMU/温控公共配置结构与查询接口。
- `imu_config.c`：采样、校准、EKF、安装矩阵与温控目标/PID/保护阈值；两车公共维护，热参数可按板上实测调整。

## 配置表结构

`app_config_t`：`vehicle_kind`、9 台 `motors[]`、`chassis`、`lift`、`arm`、`gimbal`、
`shooting`、`remote`、`power`；未适用/未确认机构整段 invalid。

## 有效性标记规则

- `configured` 是整机总开关，确认完毕才置 true。
- 分量标记 `bus_valid / id_valid / msg_id_valid / control_mode_valid / direction_valid /
  gear_ratio_valid / zero_offset_valid / limit_valid` 缺一不可。
- 机构级标记（如 `arm.valid`、`arm.limits_valid`）须与电机项同时为真。
- 占位值不生效：模块用 `app_config_status()`、`app_config_motor_motion_usable()` 判断，不拿占位值换算。

## 本车 9 个电机

| 角色 | 配置名 | 型号 |
| --- | --- | --- |
| TRACK_LEFT / TRACK_RIGHT | track_left / track_right | 3508 |
| LIFT_FRONT / LIFT_REAR | lift_front / lift_rear | GM6020 |
| ARM_YAW / ARM_SHOULDER | arm_yaw / arm_shoulder | 4310 |
| ARM_ELBOW / ARM_WRIST_ROLL | arm_elbow / arm_wrist_roll | 3508 |
| ARM_GRIPPER | arm_gripper | 2006 |

当前全部 `configured=false`；4310 为达妙电机，不能复用 DJI 驱动。

## 待确认项

总线、电机 ID、收发报文 ID、控制模式、方向、减速比、零位、限位、PID；连杆/坐标系/可达半径；
遥控通道/开关字段与 42 ms 失联时长的上板核验；功率来源；抬升行程与同步；履带轮距/轮径/最高速度。

IMU 温控参数在 `imu_config.c` 的 `temperature` 段：目标 40 °C、PWM 上限 0.8、PID
0.5/0.01/0、积分限幅 0.15、就绪 ±1 °C/3 s、预热 120 s、过期 300 ms、过温 50 °C、
升温异常 +3 °C/5 s（加热开启时）、校准/运行容差 2/3 °C、校准超时 60 s。初始值来自参考工程并加保守保护，
**加热器功率、热惯性、PWM 极性和目标温度均未上板确认**。

## 如何逐项启用

每确认一项就打开对应 `valid_*` 填真实值；单台必需项齐全后置 `configured=true`；全部确认后 `app_config_init()` 才返回 `OK`。

## 公开接口

| 接口 | 说明 |
| --- | --- |
| `app_config_init()` | 校验并缓存缺项报告；只读、不创建对象、不发报文 |
| `app_config_status()` | 整体 `UNCONFIGURED` / `NOT_READY` / `OK` |
| `app_config_motor(slot)` / `app_config_motor_missing(slot)` | 单电机配置 / 缺项位标志 |
| `app_config_motor_comms_usable(slot)` | 通信必需项齐全（决定是否注册） |
| `app_config_motor_motion_usable(slot)` | 机构运动必需项齐全（决定是否输出） |
| `app_config_find_motor(role)` | 角色 → 下标；未找到返回 -1 |
| `app_config_vehicle_kind()` | 本车种类（`ENGINEER` / `TURRET`） |

## 故障与默认行为

- 全部未确认：整体 `UNCONFIGURED`；部分确认：`NOT_READY`；电机与机构全部齐全：`OK`。
- 缺项电机不注册、不参与输出；占位数值不会被当成有效参数。
- 本模块只读，不改外设、不构造电机对象、不写 CAN。

## 调试步骤

1. 读 `app_config_report()`，按 `missing[]` 位标志逐项补配置。
2. 单台电机 `comms_usable==true` 才可能注册；`motion_usable==true` 才可能输出。
3. 只改本车 `app_config.c`；`app_config.h` 两车保持一致，不要按车改接口。

## 负责人

待认领（本车配置/机构负责人）。

## 未实现项

- 全部真实参数待确认（见上文待确认项）。
- 遥控段已启用（DBUS：USART3 + DMA/IDLE）：`channel_forward/turn/aux1/aux2`、四个方向符号、
  `gear_switch`/`enable_switch`、`link_timeout_ms=42`、`channel_deadband=0.03`；
  初始值来自协议手册与参考工程，需上板逐项核验后再改。
- 达妙协议、功率来源尚未接入，相关配置项保持 invalid。

## 职责与调用顺序

本目录唯一职责是提供本车只读配置与有效性查询；不驱动外设、不创建对象、不写 CAN。
调用顺序：`app_config_init()` 必须最先执行（`Task/app_task.c` 的应用初始化第一步），
其它模块在 `init/update` 中通过查询接口读取配置；配置在运行期视为只读，重启后才重新校验。



遥控映射新增 `hardware_verified`：`valid/mapping_valid` 只表示参数结构完整，`hardware_verified=false` 时 App_Control 不允许进入运动许可。不要在未上板核验时改成 true。

## 功率/能量配置

`power` 段的 `source_bus/source_bus_valid`、`source_timeout_ms/source_timeout_valid`、`hardware_verified` 分别控制 0x31 来源、超时和上板核验；`chassis_power_limit_w`、`buffer_energy_limit_j`、`low_buffer_energy_j` 是规则值及其独立标记。未知总线保持 `APP_CAN_BUS_NONE`、`source_bus_valid=false`；不要打开核验标记来声称实车可读。
