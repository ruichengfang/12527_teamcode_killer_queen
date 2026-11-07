# 开发进度

## 2026-11-07 当前进度

1. 主程序：遥控入口由 `MecanumDrive` 更名为 `RobotTeleOp`；驾驶站显示名称仍为 `Mecanum Drive`。
2. 底盘：由 `DriveCommand` 读取手柄，`DriveSubsystem` 管理 Pinpoint 和四个底盘电机；支持场地方向驱动及左摇杆按下重置驾驶方向。底盘自瞄已移除，旋转由右摇杆控制。
3. 云台：`servo` 更名为 `TurretSubsystem`，根据 Pinpoint 航向补偿车体旋转；Y/B 控制两个舵机位置。独立的 `TurretAim` 尚未接入。
4. 进料：`inTake` 更名为 `IntakeSubsystem`；主程序仍初始化两个进料电机，控制调用未启用。
5. 结构：子系统包更名为 `subsystem`，主循环直接执行命令，未接入 FTCLib 调度器；硬件配置名称未变。
6. 发射和自动：`Shooter`、`Blocker` 和 `AutoPath` 仍为注释代码，未启用。自动路线草稿引用已删除的底盘自瞄，不能直接启用。
7. 文档：已更新中英文 README 和程序说明，包含结构、按键、设备名及参数。
8. 验证：上次 Java 运行命令长时间无输出，编译和实机验证尚未完成；历史记录中“编译通过”不适用于当前版本。

### 当前待处理

- 完成编译及实机验证，核对 Pinpoint 偏移、起始位置、车轮方向和云台机械行程。
- 如恢复进料控制，处理 `IntakeSubsystem.constant()` 不设置 transit 电机功率的问题。
- 云台补偿跨越 ±180° 时目标可能跳变，需结合实际机械设计处理。

## 文件地址

```
D:\study\FRC\FtcRobotController\TeamCode\src\main\java\org\firstinspires\ftc\teamcode
```

## 2026-11-04 历史记录（不代表当前状态）

1. 底盘：Mecanum 场地方向驱动 + Pinpoint(od) 定位；右 Bumper 自瞄回正 0°。
2. 发射：shooter 飞轮 0/1 PID 控速，电机 2/3 供弹，已接入主程序。
3. 入料：intake 与 transit 电机合并到 inTake 子系统，同步正反转。
4. 云台：servo IMU 反向补偿；AutoAim 自瞄时新增云台回中（0.5）。
5. 自动：AutoPath 已整体注释，暂停使用。
6. 所有设备名统一在 NameConstants 管理。
7. 编译通过。

### 当时记录的问题（以当前进度为准）

1. ~~MecanumDrive intake 初始化被注释，运行空指针闪退~~（2026-11-05 已修复）
2. 右 Bumper 同时触发自瞄和 shooter 供弹，按键冲突。
3. 主循环的 servo.update 会覆盖 AutoAim 的云台回中。
4. inTake.constant() 只降速 intake 电机，transit 电机会保持上次功率不停转。
