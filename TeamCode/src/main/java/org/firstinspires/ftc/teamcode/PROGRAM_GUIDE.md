# 程序说明 / Program Guide

## 结构与状态 / Structure and status

| 文件 / File | 职责 / Responsibility | 状态 / Status |
| --- | --- | --- |
| `RobotTeleOp.java` | TeleOp 入口，初始化并运行循环 / TeleOp entry and loop | 启用 / Active |
| `commands/DriveCommand.java` | 读取 gamepad1，调用底盘 / Reads gamepad1 and calls the drive | 启用 / Active |
| `mechanisms/DriveSubsystem.java` | Pinpoint、麦轮变换与电机输出 / Pinpoint, mecanum transform and motors | 启用 / Active |
| `mechanisms/TurretSubsystem.java` | 两个舵机保持启动时的场地方向 / Dual-servo compensation for the starting field heading | 启用 / Active |
| `mechanisms/IntakeSubsystem.java` | 吸入与输送电机 / Intake and transfer motors | 已初始化，控制未启用 / Initialized, controls inactive |
| `mechanisms/TurretAim.java` | 基于定位指向场地中心 / Aims toward field center using localization | 未接入 / Not connected |
| `mechanisms/Shooter.java`, `Blocker.java` | 发射和挡球占位 / Shooter and blocker placeholders | 全部注释 / Commented out |
| `AutoPath.java` | 自动路线草稿 / Autonomous draft | 全部注释 / Commented out |
| `NameConstants.java` | 硬件配置名 / Hardware configuration names | 使用中 / In use |

这是轻量命令式分层：主循环直接调用 `DriveCommand.execute()`，没有命令调度器。This is a lightweight command/subsystem separation: the loop calls `DriveCommand.execute()` directly; there is no command scheduler.

## 运行流程 / Operation

1. Init：创建进料、云台和底盘；舵机移动到中位。底盘初始化 Pinpoint 并设置起点。Init creates the intake, turret and drive; the servos move to center and Pinpoint is initialized.
2. Start：记录云台参考航向。At Start, the turret records its reference heading.
3. 循环：更新底盘航向、处理手柄、计算四轮功率，然后更新云台和遥测。Each loop updates the drive heading, applies gamepad input and motor power, then updates the turret and telemetry.
4. 停止时底盘电机设为零。Drive motors are set to zero when the loop ends.

底盘将左摇杆的场地向量按当前航向旋转到车体坐标，横移乘以 1.1，混合右摇杆旋转量，必要时按比例缩放四轮功率。左摇杆按下记录新的底盘驾驶零方向，不改变云台参考方向。The drive rotates the left-stick field vector into robot coordinates, scales strafe by 1.1, mixes in right-stick rotation and normalizes wheel power. Pressing the left stick resets only the drive heading reference.

云台用 Pinpoint 航向减去 Start 时航向，反向补偿，并按 147/150 传动比换算到 300° 舵机，最后限制在 0～1。它保持开局方向，不是瞄准场地中心。相对航向跨过 ±180° 时目标可能跳变，行程限制也可能使其无法保持方向。The turret compensates for heading changes since Start using the 147/150 gear ratio and clips commands to the 300° servo range. It holds the starting direction, not the field center. Crossing ±180° may cause a target jump, and travel limits can prevent holding the direction.

## 手柄 / Gamepad 1

| 输入 / Input | 功能 / Function |
| --- | --- |
| 左摇杆 / Left stick | 场地方向平移 / Field-centric translation |
| 右摇杆横向 / Right stick X | 手动旋转 / Manual rotation |
| 按下左摇杆 / Left stick button | 重设驾驶航向零点 / Reset drive heading reference |
| 按住 Y / Hold Y | 双舵机位置 0 / Both servos to 0 |
| 按住 B / Hold B | 双舵机位置 1 / Both servos to 1 |

Y 优先于 B；松开后恢复开局航向补偿。Y takes priority over B; releasing them resumes heading compensation.

## 配置 / Configuration

Driver Station 配置名称必须与 `NameConstants.java` 一致：`leftFrontMotor`, `rightFrontMotor`, `leftBackMotor`, `rightBackMotor`, `od`, `leftServo`, `rightServo`, `intakeMotor`, `transitMotor`。即使进料控制未启用，初始化仍要求两个进料电机存在。Hardware names must match these values; both intake motors are required at initialization even though intake controls are inactive.

Pinpoint 偏移为 (20.1, -131.12) mm，起点为 (58.5332, 133.3355) 英寸、270°；编码器方向均为 FORWARD。按实际安装和起点核对。Pinpoint offsets are (20.1, -131.12) mm, the initial pose is (58.5332, 133.3355) inches and 270°, and both encoder directions are FORWARD. Verify these against the robot and starting position.

## 命名 / Naming

Java 包 `subSystems` 改为 `mechanisms`；`servo` → `TurretSubsystem`，`inTake` → `IntakeSubsystem`，`turretAim` → `TurretAim`；已注释的 `shooter`、`blocker` 文件也改为 `Shooter`、`Blocker`。硬件配置字符串没有修改。The Java package and class names were normalized as listed; hardware configuration strings remain unchanged.

## 待注意 / Notes

- `IntakeSubsystem.constant()` 只修改 intake 功率，不会停止 transit；重新启用前需处理。It changes only intake power and does not stop transit.
- `AutoPath` 是历史草稿，包含已删除的底盘自瞄引用和其他待修内容；不能直接取消注释使用。It is a historical draft with references to removed chassis aiming and other issues; it cannot be enabled as-is.
- `progress.md` 是历史记录，可能与当前实现不一致。It is a historical log and may not reflect the current implementation.
