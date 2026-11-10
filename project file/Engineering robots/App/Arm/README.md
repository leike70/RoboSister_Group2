# 工程车机械臂（App/Arm/）

## 职责

保存五关节调试目标与末端目标请求，按配置表门控把关节目标提交给统一电机输出；
提供运动学、任务状态机的显式占位接口。不创建电机对象、不发 CAN、不阻塞。

## 关节顺序与单位

| 下标 | 关节 | 电机 | 单位 |
| --- | --- | --- | --- |
| 0 | Yaw | 4310 | rad、rad/s |
| 1 | Shoulder（大臂） | 4310 | rad、rad/s |
| 2 | Elbow（小臂） | 3508 | rad、rad/s |
| 3 | Wrist Roll | 3508 | rad、rad/s |
| 4 | Gripper（开合） | 2006 | rad、rad/s |

顺序在 `app_arm.h` 固定，不得调换；函数名带单位后缀。

## 接口

- `app_arm_init/update/stop/get_state`：生命周期与状态快照；目标仅经 `set_joint_target` 写入。
- `set_end_target`：保存末端请求（只保存、不求解）；`clear_*` 清空目标。
- `get_joint_feedback`：反馈未接入，`valid=false`，返回 `NOT_IMPLEMENTED`。
- `app_arm_kinematics_*`：正解/逆解均返回 `NOT_IMPLEMENTED`，不输出伪结果。
- `app_arm_task_request`：合法任务置 `UNSUPPORTED` 并返回 `NOT_IMPLEMENTED`。

## 调用顺序

周期任务先清电机请求，再 `app_arm_update(now_ms)`，最后 `app_motor_submit()` 统一提交。
`arm.valid==false` 时 update 清空目标且不写任何请求。

## 配置与故障行为（待办）

配置：连杆长度、可达半径、五关节限位、零位与方向、减速比、PID 全部未确认；限位未确认也不写请求。
故障：`app_arm_stop()` 只清目标；受重力，支撑/制动/自锁与掉电策略未定，**禁止把位置归零当停机**。

## 调试、状态与负责人

先单关节低幅度（≤5°）核对零位/方向/限位再联动。骨架已搭、未烧录未实车验证；IK、任务动作序列、反馈换算未实现，负责人待认领。
