package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.AimAndShootCommand;
import org.firstinspires.ftc.teamcode.command.BlockerCommand;
import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.command.ResetLocalizationCommand;
import org.firstinspires.ftc.teamcode.command.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 2026 Biobuzz 比赛手动控制入口。 */
@TeleOp(name = "KillerQueen", group = "Competition")
public class Teleop extends OpMode {
    private LocalizationSubsystem localization;
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private BlockerSubsystem blocker;

    private DriveCommand driveCommand;
    private TurretCommand turretCommand;
    private BlockerCommand blockerCommand;
    private ResetLocalizationCommand resetLocalizationCommand;
    private AimAndShootCommand aimAndShootCommand;

    private double lastTime;
    private boolean previousA, previousX, previousY, previousLeft, previousRight;
    private boolean previousLeftStick, previousUp, previousDown, previousB;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        localization = new LocalizationSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        blocker = new BlockerSubsystem(hardwareMap);

        driveCommand = new DriveCommand(drive, localization);
        turretCommand = new TurretCommand(turret, localization);
        blockerCommand = new BlockerCommand(gamepad1, blocker);
        resetLocalizationCommand = new ResetLocalizationCommand(localization, turret);
        aimAndShootCommand = new AimAndShootCommand(shooter, turret, blocker, intake,
                localization, this::getRuntime);

        telemetry.addLine("已初始化：底盘 + Pinpoint + Shooter + 云台三态 + Blocker");
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
        telemetry.addLine("【底盘】左摇杆：场地坐标移动；右摇杆左右：转向");
        telemetry.addLine("【云台】X：自瞄开/关（需 Pinpoint）；Y：朝向锁定开/关");
        telemetry.addLine("【云台】D-pad 左/右：自瞄偏移或锁定角 +2°/-2°；X/Y 互斥");
        telemetry.addLine("【云台】普通模式回中 0.5；软限位 ±90°（位置 0.2~0.8）");
        telemetry.addLine("【Shooter】A：3300 RPM 开/关");
        telemetry.addLine("【联合发射】B 单击：停底盘/Intake，自瞄稳定后 Transit 发射 3 秒");
        telemetry.addLine("【联合发射】发射中再按 B：立即取消；Start 不再复位");
        telemetry.addLine("【Blocker】LB：打开/关闭（位置需实机校准）");
        telemetry.addLine("【Blocker】D-pad 上/下：测试变量加/减 1");
        telemetry.addLine("【Intake】RB 按住：仅 Intake 全速正转，Transit 停止");
        telemetry.addLine("【Intake】RT/LT 按住：Intake + Transit 正转/反转，力度随扳机");
        telemetry.addLine("【优先级】RB > RT > LT；全部松开则 Intake + Transit 停止");
        telemetry.addLine("【复位】左摇杆按下：IMU 零向、Pinpoint (8,111,0°)、云台普通模式");
    }

    @Override
    public void start() {
        lastTime = getRuntime();
        previousA = previousX = previousY = previousLeft = previousRight = false;
        shooter.start();
        blockerCommand.start();
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
        previousLeftStick = gamepad1.left_stick_button;
        previousB = gamepad1.b;
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double deltaSeconds = Math.max(now - lastTime, 1e-4);
        lastTime = now;

        localization.update();
        if (gamepad1.b && !previousB) {
            if (CommandScheduler.getInstance().isScheduled(aimAndShootCommand)) {
                CommandScheduler.getInstance().cancel(aimAndShootCommand);
            } else {
                CommandScheduler.getInstance().schedule(aimAndShootCommand);
            }
        }
        boolean autoShooting = CommandScheduler.getInstance().isScheduled(aimAndShootCommand);
        if (autoShooting) {
            // 一键发射期间锁定底盘，忽略两个摇杆的移动输入。
            drive.stop();
        } else {
            driveCommand.update(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                    gamepad1.right_stick_x);
        }

        turretCommand.update(!autoShooting && gamepad1.x && !previousX,
                !autoShooting && gamepad1.y && !previousY,
                !autoShooting && gamepad1.dpad_left && !previousLeft,
                !autoShooting && gamepad1.dpad_right && !previousRight);
        if (turret.isAimMode() && !turret.isReachable()) {
            gamepad1.rumble(0.8, 0.8, 200);
        } else {
            gamepad1.stopRumble();
        }

        if (!autoShooting && gamepad1.a && !previousA) shooter.toggle();
        shooter.update(deltaSeconds);

        if (!autoShooting) {
            blockerCommand.update();
            if (gamepad1.dpad_up && !previousUp && !gamepad1.dpad_down) blocker.addposition();
            if (gamepad1.dpad_down && !previousDown && !gamepad1.dpad_up) blocker.minposition();
            intake.applyManualControl(gamepad1.right_bumper, gamepad1.right_trigger,
                    gamepad1.left_trigger);
        }

        if (gamepad1.left_stick_button && !previousLeftStick) {
            CommandScheduler.getInstance().cancel(aimAndShootCommand);
            resetLocalizationCommand.execute();
        }

        previousA = gamepad1.a;
        previousX = gamepad1.x;
        previousY = gamepad1.y;
        previousLeft = gamepad1.dpad_left;
        previousRight = gamepad1.dpad_right;
        previousLeftStick = gamepad1.left_stick_button;
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
        previousB = gamepad1.b;

        CommandScheduler.getInstance().run();

        updateTelemetry();
    }

    private void updateTelemetry() {
        double yawDeg = localization.getYawDegrees();
        String turretMode = turret.isAimMode() ? "自瞄 (X 关闭)"
                : turret.isLockMode() ? "锁定 (Y 关闭)"
                : "常规 0.5 (X自瞄/Y锁定)";

        telemetry.addData("机器人场地 Yaw (°)", "%.1f", yawDeg);
        telemetry.addData("底盘 FL/BL/FR/BR", "%.2f / %.2f / %.2f / %.2f",
                drive.getFrontLeftPower(), drive.getBackLeftPower(),
                drive.getFrontRightPower(), drive.getBackRightPower());

        telemetry.addData("Pinpoint 状态", localization.getPinpointStatus());
        if (localization.isPinpointReady()) {
            telemetry.addData("Pinpoint X/Y (in)", "(%.2f, %.2f)",
                    localization.getXIn(), localization.getYIn());
            telemetry.addData("Pinpoint Heading (°)", "%.1f", localization.getHeadingDeg());
            telemetry.addData("Pinpoint - IMU 差值 (°)", "%.1f",
                    TurretSubsystem.normalizeDegrees(
                            localization.getHeadingDeg() - localization.getImuYawDegrees()));
        }

        telemetry.addData("云台状态", turretMode);
        telemetry.addData("云台舵机位置", "%.3f", turret.getPosition());
        telemetry.addData("云台软限位", "±%.0f°（位置 %.2f~%.2f）",
                TurretSubsystem.SOFT_LIMIT_DEG,
                turret.getMinServoPosition(), turret.getMaxServoPosition());
        telemetry.addData("云台目标绝对角 (°)", "%.1f", turret.getTargetYawDeg());
        telemetry.addData("云台实际绝对角 (°)", "%.1f", turret.getActualAbsoluteDeg());
        telemetry.addData("云台锁定误差 (°)", "%.1f", turret.getYawErrorDeg());
        telemetry.addData("云台可达?", turret.isReachable() ? "是" : "否（超 ±90° 软限位）");

        if (turret.isAimMode()) {
            telemetry.addData("当前瞄点 (in)", "(%.1f, %.1f)",
                    turret.getAimTargetX(), turret.getAimTargetY());
            telemetry.addData("自瞄距离 (in)", "%.2f", turret.getAimDistance());
            telemetry.addData("指向瞄点绝对角 (°)", "%.1f", turret.getAimTargetDeg());
            telemetry.addData("自瞄偏移角 (°)", "%.1f", turret.getAimOffsetDeg());
            telemetry.addData("有效瞄准已建立?", turret.hasValidAim() ? "是" : "否");
        }

        telemetry.addData("Blocker testposition（目标值）", "%.2f", blocker.getPosition());
        telemetry.addData("联合发射", aimAndShootCommand.getState());

        telemetry.addData("Shooter 目标 RPM", "%.0f", shooter.getTargetRpm());
        telemetry.addData("Shooter 实际 RPM", "%.0f", shooter.getFilteredRpm());
        telemetry.addData("Shooter 误差 RPM", "%.0f", shooter.getError());
        telemetry.addData("Shooter 功率", "%.3f", shooter.getPower());
        telemetry.addData("Shooter 状态", shooter.isRunning() ? "开 (A 关闭)" : "关 (A 开启)");

        telemetry.addData("Intake 功率", "%.2f", intake.getIntakePower());
        telemetry.addData("Transit 功率", "%.2f", intake.getTransitPower());

        if (turret.isAimMode() && !turret.isReachable()) {
            telemetry.addLine("※ 自瞄需要云台超出 ±90° 软限位，已震动提醒");
        }
        if (turret.isAimMode() && !turret.hasValidAim()) {
            telemetry.addLine("※ 距离瞄点 ≤0.5 英寸，方向未定义；请移开后再自瞄");
        }
        if (shooter.isRunning() && Math.abs(shooter.getError()) > 200.0) {
            telemetry.addLine("※ Shooter RPM 未稳定，建议等到误差 < 200 再发射");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().cancel(aimAndShootCommand);
        drive.stop();
        intake.stop();
        shooter.stop();
        shooter.update(1e-4);
        gamepad1.stopRumble();
        CommandScheduler.getInstance().reset();
    }
}
