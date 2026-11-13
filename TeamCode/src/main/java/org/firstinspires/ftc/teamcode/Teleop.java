package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.TeleopControlCommand;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShovelSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PitchSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 2026 Biobuzz 比赛手动控制入口。 */
@TeleOp(name = "Solo", group = "Competition")
@Config
public class Teleop extends OpMode {
    public static double wallShotKp = 0.000360000;
    public static double wallShotKi = 0.0;
    public static double wallShotKd = 0.000010000;
    private LocalizationSubsystem localization;
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private BlockerSubsystem blocker;
    private ShovelSubsystem shovel;
    private PitchSubsystem pitch;

    private TeleopControlCommand controls;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        localization = new LocalizationSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, () -> wallShotKp, () -> wallShotKi, () -> wallShotKd);
        turret = new TurretSubsystem(hardwareMap, false);
        blocker = new BlockerSubsystem(hardwareMap);
        shovel = new ShovelSubsystem(hardwareMap);
        pitch = new PitchSubsystem(hardwareMap);

        controls = new TeleopControlCommand(gamepad1, gamepad2, this::getRuntime,
                localization, drive, intake, shooter, turret, blocker, shovel, pitch);

        telemetry.addLine(localization.isPinpointReady() ? "Pinpoint: 已启用" : "Pinpoint: 未找到");
        telemetry.addLine(shovel.isAvailable() ? "铲子舵机: 已连接" : "铲子舵机: 未配置 shovelServo");
        telemetry.addLine(pitch.isAvailable() ? "俯仰舵机: 已连接" : "俯仰舵机: 未配置 pitchServo");
        addControlHelp();
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // 直接读取 Pinpoint 当前位姿，自动到手动切换时不重置设备。
        localization.update();

        telemetry.addData("初始坐标来源", "Pinpoint 当前位姿");
        telemetry.addData("Pinpoint 状态", localization.getPinpointStatus());
        telemetry.addData("当前坐标有效", localization.isFieldPoseValid());
        if (localization.isFieldPoseValid()) {
            telemetry.addData("当前 X/Y (in)", "(%.2f, %.2f)",
                    localization.getXIn(), localization.getYIn());
            telemetry.addData("当前 Heading (°)", "%.1f", localization.getHeadingDeg());
        } else {
            telemetry.addLine("等待 Pinpoint READY，尚未读取有效坐标");
        }
        telemetry.update();
    }

    /** 主操负责驾驶机构，副操方向键微调 Pinpoint 坐标。 */
    private void addControlHelp() {
        telemetry.addLine("内置 IMU 无头平移（START 时车头为前方，按下左摇杆重设），右摇杆转向");
        telemetry.addLine("按 Option 将 Pinpoint 坐标设为 (72, 72 in, 180°)");
        telemetry.addLine("副操方向键：上/下 X ±1 in，右/左 Y ±1 in（每按一次）");
        telemetry.addLine("副操 LB / RB：俯仰舵机位置 − / + 0.01");
        telemetry.addLine("按住 B 用 Pinpoint 控制转向，左摇杆仍可平移");
        telemetry.addLine("主操 X：切换贴墙射 / 云台自动瞄准");
        telemetry.addLine("主操 Y：切换铲子上 / 下");
        telemetry.addLine("飞轮常转；按住 RT 射击，挡板打开后 transit 送料");
        telemetry.addData("自瞄模式飞轮转速", "%.0f TPS", ShooterSubsystem.SHOT_TICKS_PER_SECOND);
        telemetry.addData("自瞄：只按 RT 远射角度", "servo %.2f", ShooterSubsystem.SERVO_LOW);
        telemetry.addData("自瞄：RB + RT 中射 / LB + RT 近射", "servo %.2f", ShooterSubsystem.SERVO_HIGH);
        telemetry.addLine("intake 待机 0.7；按住 LT 吸球，松开停止 transit");
        telemetry.addLine("单独按住 LB：仅 intake 全速退球");
        telemetry.addLine("按住方向键下：仅 intake 全速吐球");
        telemetry.addLine("按住 RT 飞轮达速后自动开挡板，松开 RT 关闭；云台常态自瞄");
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
                ? String.format(java.util.Locale.US, "IMU 无头（相对前方 %.1f°）", Math.toDegrees(driverHeading))
                : "IMU 航向无效，底盘停止");
        telemetry.addData("底盘 FL/BL/FR/BR", "%.2f / %.2f / %.2f / %.2f",
                drive.getFrontLeftPower(), drive.getBackLeftPower(),
                drive.getFrontRightPower(), drive.getBackRightPower());

        telemetry.addData("Pinpoint 状态", localization.getPinpointStatus());
        telemetry.addData("副操坐标微调", controls.getTrimStatus());
        if (localization.isPinpointReady()) {
            telemetry.addData("Pinpoint X/Y (in)", "(%.2f, %.2f)",
                    localization.getXIn(), localization.getYIn());
            telemetry.addData("Pinpoint Heading (°)", "%.1f", localization.getHeadingDeg());
            telemetry.addData("Pinpoint - IMU 差值 (°)", "%.1f",
                    TurretSubsystem.normalizeDegrees(localization.getHeadingDeg() - localization.getImuYawDegrees()));
        }

        telemetry.addData("云台位置", "%.3f", turret.getPosition());
        telemetry.addData("云台模式", controls.isTurretCentered() ? "贴墙射：锁定中位" : "自动瞄准");
        telemetry.addData("云台目标", controls.isTurretCentered() ? "中位锁定"
                : !localization.isFieldPoseValid() ? "等待有效定位"
                : !turret.hasValidAim() ? "无有效目标" : turret.isReachable() ? "在限位内" : "超出限位");
        telemetry.addData("云台估计瞄准", controls.isTurretAimed() ? "是（无位置反馈）" : "否");
        telemetry.addData("挡板", "%s (%.2f)", blocker.isOpen() ? "开" : "关", blocker.getPosition());
        telemetry.addData("铲子", !shovel.isAvailable() ? "未配置 shovelServo"
                : !shovel.hasTarget() ? "等待操作" : shovel.isShovelUp() ? "上" : "下");
        if (pitch.isAvailable()) telemetry.addData("俯仰舵机位置", "%.2f", pitch.getPosition());
        else telemetry.addLine("俯仰舵机未配置 pitchServo");
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
        telemetry.addData("射击档位", "%s：servo %.3f",
                controls.isTurretCentered() ? "贴墙射" : shooter.getShotModeName(), shooter.getServoPosition());
        if (!shooter.getControlFault().isEmpty()) telemetry.addData("飞轮控制故障", shooter.getControlFault());
        telemetry.addData("Shooter 状态", shooter.isRunning() ? "开（START 后常转）" : "关");

        telemetry.addData("Intake 功率", "%.2f", intake.getIntakePower());
        telemetry.addData("Transit 功率", "%.2f", intake.getTransitPower());
        telemetry.addData("LT 吸球请求", controls.isIntakeEnabled() ? "开" : "关");

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
