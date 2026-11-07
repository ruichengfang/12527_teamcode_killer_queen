# TeamCode / 机器人代码

## 中文

本模块是 FTC 机器人控制代码。驾驶站选择 **Mecanum Drive**，通过 Pinpoint 实现场地方向驾驶，并使用双舵机补偿车体旋转。底盘不包含自动瞄准。

代码分为主程序、驾驶命令和子系统：`RobotTeleOp` → `DriveCommand` → `DriveSubsystem`；主程序同时调用 `TurretSubsystem`。命令由主循环直接执行，本项目没有使用 FTCLib 调度器。

完整流程、按键、设备名、参数与未启用功能见 [程序说明](PROGRAM_GUIDE.md)，端口记录见 [接线记录](wiring.md)。上机前请核对实际设备配置、方向和机械行程。

## English

This module contains the FTC robot code. Select **Mecanum Drive** on the Driver Station for field-centric driving using Pinpoint and dual-servo turret heading compensation. The drivetrain has no automatic aiming.

The code is organized as an OpMode, a drive command, and subsystems: `RobotTeleOp` → `DriveCommand` → `DriveSubsystem`. The OpMode also calls `TurretSubsystem`. The main loop invokes the command directly; there is no FTCLib scheduler.

See the [program guide](PROGRAM_GUIDE.md) for behavior, controls, hardware names, parameters and inactive features, and the [wiring record](wiring.md) for ports. Verify the hardware configuration, directions and mechanical travel before operating.
