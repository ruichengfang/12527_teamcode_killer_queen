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
    // 测试专用。手柄编辑最近的实测点，运行时保存在射击表中。
    private static final double RPM_STEP = 100.0;
    private static final double FINE_RPM_STEP = 10.0;
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
    private double[] calibrationPoint;
    private double requestedRpm;
    private double feedReferenceRpm, feedReferencePitch;
    private double lastLoopTime;
    private boolean previousUp, previousDown, previousLeft, previousRight;
    private boolean previousLb, previousRb, previousLeftStick, previousRightStick;

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
        shooter.start();
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
        // 测试专用。右摇杆按下沿用比赛手动的 OD 中心重置。
        if (gamepad1.right_stick_button && !previousRightStick) localization.resetToFieldCenter();
        drive.execute();

        // 测试专用。固定使用分段自瞄，沿用比赛云台的 Hive 选择和限位。
        if (!turret.isAimMode() && localization.isFieldPoseValid()) turret.setAimMode(true, true);
        turret.update(localization);
        boolean shooting = gamepad1.right_trigger > ShootConstants.TRIGGER_THRESHOLD;
        centerBlocked = SegmentedAutoAim.isInCenter(localization);
        double aimDistance = !centerBlocked && localization.isFieldPoseValid() && turret.hasValidAim()
                ? turret.getAimDistance() : Double.NaN;
        calibrationPoint = SegmentedAutoAim.findNearestCalibrationPoint(aimDistance);
        boolean settingsEdited = updateTestSettings(shooting);
        shotProfile = SegmentedAutoAim.findProfileForDistance(aimDistance);
        requestedRpm = shotProfile == null ? shooter.getShotRpm()
                : Math.min(shotProfile[SegmentedAutoAim.RPM], MAX_TARGET_RPM);
        if (shotProfile != null) pitch.setPosition(shotProfile[SegmentedAutoAim.PITCH]);
        boolean targetChanged = blocker.isOpen() && SegmentedAutoAim.hasFeedTargetChanged(
                requestedRpm, pitch.getPosition(), feedReferenceRpm, feedReferencePitch);
        updateShot(shooting, settingsEdited || targetChanged, dt);
        intake.execute();
        rememberButtons();
        showTelemetry();
    }

    /** 测试专用。直接编辑最近实测点，按住 RT 时跳过手动 pitch 微调。 */
    private boolean updateTestSettings(boolean shooting) {
        if (calibrationPoint == null) return false;
        double previousRpm = calibrationPoint[SegmentedAutoAim.RPM];
        double previousPitch = calibrationPoint[SegmentedAutoAim.PITCH];
        double rpmAdjustment = getRpmAdjustment();
        if (rpmAdjustment != 0.0) {
            calibrationPoint[SegmentedAutoAim.RPM] = Range.clip(
                    previousRpm + rpmAdjustment, MIN_TARGET_RPM, MAX_TARGET_RPM);
        }

        double pitchStep = PitchSubsystem.pitchStep;
        if (!shooting && Double.isFinite(pitchStep) && pitchStep >= 0.0) {
            if (gamepad1.left_bumper && !previousLb && !gamepad1.right_bumper) {
                calibrationPoint[SegmentedAutoAim.PITCH] = Range.clip(previousPitch - pitchStep, 0.0, 1.0);
            } else if (gamepad1.right_bumper && !previousRb && !gamepad1.left_bumper) {
                calibrationPoint[SegmentedAutoAim.PITCH] = Range.clip(previousPitch + pitchStep, 0.0, 1.0);
            }
        }
        return calibrationPoint[SegmentedAutoAim.RPM] != previousRpm
                || calibrationPoint[SegmentedAutoAim.PITCH] != previousPitch;
    }

    /** 测试专用。上下粗调，左右微调，每次按下只改一次。 */
    private double getRpmAdjustment() {
        if (gamepad1.dpad_up && !previousUp && !gamepad1.dpad_down) return RPM_STEP;
        if (gamepad1.dpad_down && !previousDown && !gamepad1.dpad_up) return -RPM_STEP;
        if (gamepad1.dpad_left && !previousLeft && !gamepad1.dpad_right) return -FINE_RPM_STEP;
        if (gamepad1.dpad_right && !previousRight && !gamepad1.dpad_left) return FINE_RPM_STEP;
        return 0.0;
    }

    private void rememberButtons() {
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
        previousLeft = gamepad1.dpad_left;
        previousRight = gamepad1.dpad_right;
        previousLb = gamepad1.left_bumper;
        previousRb = gamepad1.right_bumper;
        previousLeftStick = gamepad1.left_stick_button;
        previousRightStick = gamepad1.right_stick_button;
    }

    /** 测试专用。飞轮常转，RT 只在有有效插值参数且达速时开挡板。 */
    private void updateShot(boolean shooting, boolean targetChanged, double dt) {
        shooter.setTargetRpm(requestedRpm);
        shooter.update(dt);
        if (targetChanged || !shooting || shotProfile == null || !shooter.getControlFault().isEmpty()) {
            if (blocker.isOpen()) blocker.setOpen(false);
        } else if (!blocker.isOpen() && shooter.isReady()) {
            blocker.setOpen(true);
            blockerOpenTimer.reset();
            feedReferenceRpm = requestedRpm;
            feedReferencePitch = pitch.getPosition();
        }
        blocker.flushOutput();
    }

    /** 测试专用遥测。pitchServo 显示 0 到 1 的指令位置。 */
    private void showTelemetry() {
        telemetry.addLine("左摇杆平移，右摇杆转向，按下左摇杆重设前方");
        telemetry.addLine("按下右摇杆重置 OD 到 (72, 72 in, 180°)");
        telemetry.addLine("调最近实测点：LB / RB 调 pitch，上 / 下 ±100 RPM，左 / 右 −/+10 RPM");
        telemetry.addLine("START 后飞轮常转，LT 吸球，按住 RT 达速后射击");
        telemetry.addData("OD 状态", localization.getPinpointStatus());
        if (localization.isFieldPoseValid()) {
            telemetry.addData("OD X / Y (in)", "%.2f / %.2f", localization.getXIn(), localization.getYIn());
            telemetry.addData("OD 航向 (°)", "%.1f", localization.getHeadingDeg());
        }
        telemetry.addData("自瞄模式", "分段自瞄");
        if (shotProfile != null) {
            telemetry.addData("插值距离 / RPM / Pitch", "%s / %.2f RPM / %.3f",
                    SegmentedAutoAim.getProfileLabel(shotProfile), requestedRpm,
                    shotProfile[SegmentedAutoAim.PITCH]);
        } else {
            telemetry.addData("射击档", centerBlocked ? "中央禁射" : SegmentedAutoAim.getProfileLabel(null));
            telemetry.addData("飞轮预热目标 RPM", "%.2f", requestedRpm);
        }
        if (calibrationPoint != null) {
            telemetry.addData("编辑点 距离 / RPM / Pitch", "%s / %.2f RPM / %.3f",
                    SegmentedAutoAim.getProfileLabel(calibrationPoint), calibrationPoint[SegmentedAutoAim.RPM],
                    calibrationPoint[SegmentedAutoAim.PITCH]);
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
