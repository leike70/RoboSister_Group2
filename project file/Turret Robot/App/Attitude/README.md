# Attitude 姿态模块（两车公共）

## 职责
保存 IMU 解算后的姿态一致快照并提供状态。采集、温控与解算在独立 `IMU_Acquire` 任务中，
由 `imu_task` 在温度稳定、静止校准完成并进入 RUNNING 后调用 `app_attitude_ingest()` 发布；
本模块只做快照、数值校验和新鲜度检查。温度准备/校准/故障期间不发布可用于控制的姿态。
本模块不做安装方向修正、不做姿态控制。

## 公开接口与单位
| 接口 | 说明 |
| --- | --- |
| `app_attitude_init(now_ms)` | 清空快照为无效；返回 NOT_READY |
| `app_attitude_update(now_ms)` | 每周期调用；有效数据超过 `IMU_DATA_TIMEOUT_MS`（20 ms）未更新时清零并置 OFFLINE |
| `app_attitude_ingest(sample)` | 有效且有限才返回 OK；valid=false → NOT_READY；NaN/Inf → INVALID_PARAM；无效帧清零快照但保留时间戳 |
| `app_attitude_get_snapshot(out)` | 拷贝快照；valid=false 时角度/角速度均无意义 |
| `app_attitude_status()` | 模块状态：NOT_READY / OFFLINE / INVALID_PARAM / OK |

单位：角度 rad、角速度 rad/s；时间戳 ms，由 IMU 任务用 `app_time_now_ms()` 填写。
默认输出板上传感器坐标；车体安装矩阵未标定前不得当作车体姿态。

## 调用顺序/周期
`init` 一次 → 每个 App_Control 周期 `update(now_ms)` → 需要姿态的模块读 `get_snapshot()`。
`ingest` 由 IMU 任务在每次解算后调用（约 2 ms）。本模块不阻塞、不创建线程。

## 配置项
超时常量 `IMU_DATA_TIMEOUT_MS` 在 `Config/imu_config.h`，当前 20 ms；
采样周期、静止校准样本数、安装矩阵、EKF 参数和温度目标/保护阈值在 `Config/imu_config.c`。
本模块只读取姿态超时常量，不直接控制加热。

## 故障与默认行为
- 温控准备（TEMP_PREP）、静止校准（CALIBRATING）、IMU 初始化/重连和故障期间只发布无效快照，
  本模块保持 NOT_READY/FAULT，不把 0 当有效姿态。
- 只有进入 RUNNING 后才发布 `valid=true`；温度偏离、EKF 故障或传感器错误会立即撤销。
- 有效数据超过 20 ms 未更新：清零 roll/pitch/yaw 与角速度并置 OFFLINE。
- valid=false 或 NaN/Inf 样本会发布无效清零快照并保留该帧时间戳，使模块降级。
- NULL 仅返回 INVALID_PARAM，不改变已有快照；恢复有效采样后自动回到 OK。

快照与状态码在 `Common/app_critical.*` 的短临界区中整帧发布/读取。
输入调用期间必须稳定；普通任务/ISR 可调用，NMI/HardFault 不调用。

## 调试步骤
1. 断点看 `imu_task_get_report()`：确认 `state` 已到 RUNNING、温度/校准有效、样本数递增。
2. 再看 `app_attitude_get_snapshot()`：静止时 roll/pitch 应与摆放一致、角速度接近 0。
   `app_task_get_report()` 的 `attitude_valid` / `attitude_ms` 也反映快照状态。
3. 缓慢绕三轴转动，核对角速度符号与角度变化，确认后再标定安装矩阵。
4. 停止 IMU 任务或断传感器，确认 20 ms 后快照失效并置 OFFLINE；观察温控故障字段。

## 负责人
待认领（公共模块，C5 维护）。

## 未实现项
- 车体安装矩阵/零位标定、温度补偿、航向无漂移（六轴 IMU 无法提供）。
- 姿态闭环控制、与底盘/云台的联锁未实现。
