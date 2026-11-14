package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Teleop;
import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.shoot.SegmentedAutoAim;
import org.firstinspires.ftc.teamcode.shoot.ShootConstants;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PitchSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 测试专用手动程序，用来边驾驶边测试飞轮转速与俯仰位置。 */
@TeleOp(name = "KQ Shooter + Pitch Test", group = "Test")
public final class ShooterPitchTestTeleop extends OpMode {
    // 测试专用。手柄直接编辑当前档的参数，运行时保存在射击表中。
    private static final double RPM_STEP = 100.0;
    private static final double MIN_TARGET_RPM = 100.0;
    private static final double MAX_TARGET_RPM = 5000.0;
    private final ElapsedTime blockerOpenTimer = new ElapsedTime();

    private LocalizationSubsystem localization;
    private DriveCommand drive;
    private IntakeCommand intake;
    private ShooterSubsystem shooter;
    private BlockerSubsystem blocker;
    private PitchSubsystem pitch;
    private TurretSubsystem turret;
    private boolean centerBlocked;
    private double[] shotProfile;
    private double requestedRpm;
    private double lastLoopTime;
    private boolean previousUp, previousDown, previousLb, previousRb, previousLeftStick;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        localization = new LocalizationSubsystem(hardwareMap);
        drive = new DriveCommand(new DriveSubsystem(hardwareMap), localization,
                () -> -gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> gamepad1.right_stick_x);
        // 测试专用。用比赛手动程序的飞轮 PID 比较不同转速下的表现。
        shooter = new ShooterSubsystem(hardwareMap,
                () -> Teleop.wallShotKp, () -> Teleop.wallShotKi, () -> Teleop.wallShotKd);
        blocker = new BlockerSubsystem(hardwareMap);
        pitch = new PitchSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap, false);
        // 测试专用。LB 和方向键下由调参使用，LT 吸球和 RT 送料沿用比赛命令。
        intake = new IntakeCommand(new IntakeSubsystem(hardwareMap), gamepad1,
                () -> blocker.isOpen() && blockerOpenTimer.seconds() >= ShooterSubsystem.TRANSIT_DELAY_SECONDS,
                () -> false);
        showTelemetry();
    }

    @Override
    public void init_loop() {
        localization.update();
        showTelemetry();
    }

    @Override
    public void start() {
        rememberButtons();
        lastLoopTime = getRuntime();
        localization.resetDriverHeading();
        drive.initialize();
        intake.initialize();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = Math.max(now - lastLoopTime, 1e-4);
        lastLoopTime = now;
        localization.update();
        if (gamepad1.left_stick_button && !previousLeftStick) localization.resetDriverHeading();
        drive.execute();

        // 测试专用。固定使用分段自瞄，沿用比赛云台的 Hive 选择和限位。
        if (!turret.isAimMode() && localization.isFieldPoseValid()) turret.setAimMode(true, true);
        turret.update(localization);
        boolean shooting = gamepad1.right_trigger > ShootConstants.TRIGGER_THRESHOLD;
        centerBlocked = SegmentedAutoAim.isInCenter(localization);
        shotProfile = centerBlocked ? null : SegmentedAutoAim.select(localization, turret);
        double previousRpm = requestedRpm;
        double previousPitch = pitch.getPosition();
        updateTestSettings(shooting);
        updateShot(shooting, requestedRpm != previousRpm
                || pitch.getPosition() != previousPitch, dt);
        intake.execute();
        rememberButtons();
        showTelemetry();
    }

    /** 测试专用。直接改当前距离档的 RPM 和 pitch，按住 RT 时保持 pitch。 */
    private void updateTestSettings(boolean shooting) {
        if (shotProfile == null) {
            requestedRpm = 0.0;
            return;
        }
        double tableRpm = shotProfile[SegmentedAutoAim.RPM];
        if (gamepad1.dpad_up && !previousUp && !gamepad1.dpad_down) {
            shotProfile[SegmentedAutoAim.RPM] = Range.clip(tableRpm + RPM_STEP, MIN_TARGET_RPM, MAX_TARGET_RPM);
        } else if (gamepad1.dpad_down && !previousDown && !gamepad1.dpad_up) {
            shotProfile[SegmentedAutoAim.RPM] = Range.clip(tableRpm - RPM_STEP, MIN_TARGET_RPM, MAX_TARGET_RPM);
        }
        requestedRpm = Math.min(shotProfile[SegmentedAutoAim.RPM], MAX_TARGET_RPM);

        double pitchStep = PitchSubsystem.pitchStep;
        if (!shooting && Double.isFinite(pitchStep) && pitchStep >= 0.0) {
            double tablePitch = shotProfile[SegmentedAutoAim.PITCH];
            if (gamepad1.left_bumper && !previousLb && !gamepad1.right_bumper) {
                shotProfile[SegmentedAutoAim.PITCH] = Range.clip(tablePitch - pitchStep, 0.0, 1.0);
            } else if (gamepad1.right_bumper && !previousRb && !gamepad1.left_bumper) {
                shotProfile[SegmentedAutoAim.PITCH] = Range.clip(tablePitch + pitchStep, 0.0, 1.0);
            }
        }
        pitch.setPosition(shotProfile[SegmentedAutoAim.PITCH]);
    }

    private void rememberButtons() {
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
        previousLb = gamepad1.left_bumper;
        previousRb = gamepad1.right_bumper;
        previousLeftStick = gamepad1.left_stick_button;
    }

    /** 测试专用。RT 使用当前分段转速，中央禁射，达速后开挡板。 */
    private void updateShot(boolean shooting, boolean targetChanged, double dt) {
        boolean runFlywheel = shooting && requestedRpm > 0.0;
        shooter.setTargetRpm(runFlywheel ? requestedRpm : 0.0);
        shooter.update(dt);
        if (targetChanged || !runFlywheel || !shooter.getControlFault().isEmpty()) {
            if (blocker.isOpen()) blocker.setOpen(false);
        } else if (!blocker.isOpen() && shooter.isReady()) {
            blocker.setOpen(true);
            blockerOpenTimer.reset();
        }
        blocker.flushOutput();
    }

    /** 测试专用遥测。pitchServo 显示 0 到 1 的指令位置。 */
    private void showTelemetry() {
        telemetry.addLine("左摇杆平移，右摇杆转向，按下左摇杆重设前方");
        telemetry.addLine("主操 LB / RB 改当前档 pitch，方向键上 / 下改当前档 RPM");
        telemetry.addLine("LT 吸球，按住 RT 升速并射击");
        telemetry.addData("OD 状态", localization.getPinpointStatus());
        if (localization.isFieldPoseValid()) {
            telemetry.addData("OD X / Y (in)", "%.2f / %.2f", localization.getXIn(), localization.getYIn());
            telemetry.addData("OD 航向 (°)", "%.1f", localization.getHeadingDeg());
        }
        telemetry.addData("自瞄模式", "分段自瞄");
        if (shotProfile != null) {
            telemetry.addData("当前档 距离 / RPM / Pitch", "%.1f in / %.2f RPM / %.3f",
                    shotProfile[SegmentedAutoAim.DISTANCE], shotProfile[SegmentedAutoAim.RPM],
                    shotProfile[SegmentedAutoAim.PITCH]);
            telemetry.addData("测试目标 RPM", "%.2f", requestedRpm);
        } else {
            telemetry.addData("射击档", centerBlocked ? "中央禁射" : SegmentedAutoAim.getProfileLabel(null));
        }
        if (turret.isAimMode() && turret.hasValidAim()) {
            telemetry.addData("目标 Hive 水平距离 (in)", "%.1f", turret.getAimDistance());
        }
        if (!pitch.isAvailable()) telemetry.addLine("未配置 pitchServo");
        telemetry.addData("Shooter 实际 RPM", "%.0f", shooter.getFilteredRpm());
        telemetry.addData("飞轮达速", shooter.isReady() ? "是" : "否");
        if (!shooter.getControlFault().isEmpty()) telemetry.addData("飞轮故障", shooter.getControlFault());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (blocker != null) blocker.setOpen(false);
        if (intake != null) intake.end(true);
        if (drive != null) drive.end(true);
        if (shooter != null) {
            shooter.stop();
            shooter.update(1e-4);
        }
        CommandScheduler.getInstance().reset();
    }
}
