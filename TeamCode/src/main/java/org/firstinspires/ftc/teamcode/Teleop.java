package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.TeleopControlCommand;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 2026 Biobuzz 比赛手动控制入口。 */
@TeleOp(name = "Solo", group = "Competition")
public class Teleop extends OpMode {
    private LocalizationSubsystem localization;
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private BlockerSubsystem blocker;

    private TeleopControlCommand controls;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        localization = new LocalizationSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap, true);
        blocker = new BlockerSubsystem(hardwareMap);

        controls = new TeleopControlCommand(gamepad1, this::getRuntime,
                localization, drive, intake, shooter, turret, blocker);

        telemetry.addLine(localization.isPinpointReady() ? "Pinpoint: 已启用" : "Pinpoint: 未找到");
        addControlHelp();
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // INIT 可以复位 Pinpoint；READY 后才恢复自动末位置，并在写入后重新读取。
        localization.update();

        telemetry.addData("初始坐标来源", AutoEndPose.hasValue() ? "自动末位置" : "手动默认起点");
        telemetry.addData("Pinpoint 状态", localization.getPinpointStatus());
        telemetry.addData("初始坐标已写入", localization.isPositionInitialized());
        if (localization.isPositionInitialized()) {
            telemetry.addData("当前 X/Y (in)", "(%.2f, %.2f)",
                    localization.getXIn(), localization.getYIn());
            telemetry.addData("当前 Heading (°)", "%.1f", localization.getHeadingDeg());
        } else if (AutoEndPose.hasValue()) {
            telemetry.addData("待恢复 X/Y (in)", "(%.2f, %.2f)",
                    AutoEndPose.getXIn(), AutoEndPose.getYIn());
            telemetry.addData("待恢复 Heading (°)", "%.1f",
                    Math.toDegrees(AutoEndPose.getHeadingRad()));
            telemetry.addLine("等待 Pinpoint READY，尚未读取有效坐标");
        }
        telemetry.update();
    }

    /** 完整按键绑定：所有控制均使用 Gamepad1，Gamepad2 未使用。 */
    private void addControlHelp() {
        telemetry.addLine("左摇杆无头平移（START 时车头为前方，按下左摇杆重设），右摇杆转向");
        telemetry.addLine("按住 B 用 Pinpoint 控制转向，左摇杆仍可平移");
        telemetry.addLine("按住 RT 转飞轮，intake + transit 全速");
        telemetry.addData("只按 RT 远射", "servo %.2f / %.0f TPS",
                ShooterSubsystem.SERVO_LOW, ShooterSubsystem.FAR_TICKS_PER_SECOND);
        telemetry.addData("RB + RT 中射", "servo %.2f / %.0f TPS",
                ShooterSubsystem.SERVO_MID, ShooterSubsystem.MID_TICKS_PER_SECOND);
        telemetry.addData("LB + RT 近射", "servo %.2f / %.0f TPS",
                ShooterSubsystem.SERVO_HIGH, ShooterSubsystem.NEAR_TICKS_PER_SECOND);
        telemetry.addLine("按住 LT 全速吸入");
        telemetry.addLine("按住方向键下全速吐球，优先于吸入");
        telemetry.addLine("按住 RT 飞轮达速后自动开挡板，松开 RT 关闭；云台固定中位");
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(controls);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        updateTelemetry();
    }

    private void updateTelemetry() {
        double yawDeg = localization.getYawDegrees();
        telemetry.addData("机器人场地 Yaw (°)", "%.1f", yawDeg);
        double driverHeading = localization.getManualDriveHeadingRadians();
        telemetry.addData("底盘锁定", controls.getDriveCommand().isHolding() ? "电机刹车锁死" : "否");
        telemetry.addData("底盘模式", Double.isFinite(driverHeading)
                ? String.format(java.util.Locale.US, "无头（相对前方 %.1f°）", Math.toDegrees(driverHeading))
                : "车头坐标（Pinpoint 不可用）");
        telemetry.addData("底盘 FL/BL/FR/BR", "%.2f / %.2f / %.2f / %.2f",
                drive.getFrontLeftPower(), drive.getBackLeftPower(),
                drive.getFrontRightPower(), drive.getBackRightPower());

        telemetry.addData("Pinpoint 状态", localization.getPinpointStatus());
        if (localization.isPinpointReady()) {
            telemetry.addData("Pinpoint X/Y (in)", "(%.2f, %.2f)",
                    localization.getXIn(), localization.getYIn());
            telemetry.addData("Pinpoint Heading (°)", "%.1f", localization.getHeadingDeg());
            telemetry.addData("Pinpoint - IMU 差值 (°)", "%.1f",
                    TurretSubsystem.normalizeDegrees(localization.getHeadingDeg() - localization.getImuYawDegrees()));
        }

        telemetry.addData("云台中位", "%.3f", turret.getPosition());
        telemetry.addData("挡板", "%s (%.2f)", blocker.isOpen() ? "开" : "关", blocker.getPosition());
        telemetry.addData("底盘转向", controls.isAiming() ? "Pinpoint 自瞄" : "右摇杆");
        if (controls.isAiming()) {
            telemetry.addData("瞄点有效", controls.getAimTarget().hasValidAimingTarget());
            telemetry.addData("瞄准误差 (°)", "%.1f", controls.getAimTarget().getYawErrorDegrees());
            telemetry.addData("底盘已对准", controls.getAutoAim().isLocked());
        }
        telemetry.addData("飞轮达速", shooter.isReady() ? "是" : "否");

        telemetry.addData("Shooter 目标 RPM", "%.0f", shooter.getTargetRpm());
        telemetry.addData("Shooter 实际 RPM", "%.0f", shooter.getFilteredRpm());
        telemetry.addData("飞轮编码器", shooter.getEncoderName());
        telemetry.addData("Shooter 误差 RPM", "%.0f", shooter.getError());
        telemetry.addData("Shooter 功率", "%.3f", shooter.getPower());
        telemetry.addData("射击档位", "%s：servo %.3f / 目标 %.0f TPS",
                shooter.getShotModeName(), shooter.getServoPosition(),
                shooter.getShotTicksPerSecond());
        if (!shooter.getControlFault().isEmpty()) telemetry.addData("飞轮控制故障", shooter.getControlFault());
        telemetry.addData("Shooter 状态", shooter.isRunning() ? "开（松 RT 停止）" : "关（按住 RT 启动）");

        telemetry.addData("Intake 功率", "%.2f", intake.getIntakePower());
        telemetry.addData("Transit 功率", "%.2f", intake.getTransitPower());

        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        if (scheduler.isScheduled(controls)) {
            scheduler.cancelAll();
        } else {
            // INIT 后直接 STOP 时组合命令尚未启动。
            drive.stop();
            intake.stop();
            shooter.stop();
            shooter.update(1e-4);
            blocker.setOpen(false);
        }
        scheduler.reset();
    }
}
