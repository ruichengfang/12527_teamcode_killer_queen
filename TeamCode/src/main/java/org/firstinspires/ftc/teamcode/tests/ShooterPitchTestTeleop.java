package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Teleop;
import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.shoot.ShootConstants;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PitchSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

/** 测试专用手动程序，用来边驾驶边测试飞轮转速与俯仰位置。 */
@TeleOp(name = "KQ Shooter + Pitch Test", group = "Test")
public final class ShooterPitchTestTeleop extends OpMode {
    // 测试专用目标转速以 RPM 调整，仅在本次 OpMode 内保存。
    private static final double RPM_STEP = 100.0;
    private static final double MAX_TARGET_RPM = 5000.0;
    private final ElapsedTime blockerOpenTimer = new ElapsedTime();

    private LocalizationSubsystem localization;
    private DriveCommand drive;
    private IntakeCommand intake;
    private ShooterSubsystem shooter;
    private BlockerSubsystem blocker;
    private PitchSubsystem pitch;
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
        // 测试专用。LB 和方向键下由调参使用，LT 吸球和 RT 送料沿用比赛命令。
        intake = new IntakeCommand(new IntakeSubsystem(hardwareMap), gamepad1,
                () -> blocker.isOpen() && blockerOpenTimer.seconds() >= ShooterSubsystem.TRANSIT_DELAY_SECONDS,
                () -> false);
        requestedRpm = Range.clip(shooter.getShotRpm(), 0.0, MAX_TARGET_RPM);
        showTelemetry();
    }

    @Override
    public void init_loop() {
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
        if (gamepad1.left_stick_button && !previousLeftStick) localization.resetDriverHeading();
        drive.execute();

        boolean shooting = gamepad1.right_trigger > ShootConstants.TRIGGER_THRESHOLD;
        double previousRpm = requestedRpm;
        updateTestSettings(shooting);
        updateShot(shooting, requestedRpm != previousRpm, dt);
        intake.execute();
        rememberButtons();
        showTelemetry();
    }

    /** 测试专用。主操方向键上下调转速，LB/RB 微调 pitch，按住 RT 时保持 pitch。 */
    private void updateTestSettings(boolean shooting) {
        if (gamepad1.dpad_up && !previousUp && !gamepad1.dpad_down) {
            requestedRpm = Math.min(requestedRpm + RPM_STEP, MAX_TARGET_RPM);
        } else if (gamepad1.dpad_down && !previousDown && !gamepad1.dpad_up) {
            requestedRpm = Math.max(requestedRpm - RPM_STEP, 0.0);
        }

        if (!shooting) {
            if (gamepad1.left_bumper && !previousLb && !gamepad1.right_bumper) pitch.adjust(-1);
            if (gamepad1.right_bumper && !previousRb && !gamepad1.left_bumper) pitch.adjust(1);
        }
    }

    private void rememberButtons() {
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
        previousLb = gamepad1.left_bumper;
        previousRb = gamepad1.right_bumper;
        previousLeftStick = gamepad1.left_stick_button;
    }

    /** 测试专用。RT 使用预设转速发射，达速后开挡板，发射期间不切换射击角度档位。 */
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
        telemetry.addLine("主操 LB / RB 调 pitch，方向键上 / 下调转速");
        telemetry.addLine("LT 吸球，按住 RT 升速并射击");
        if (pitch.isAvailable()) telemetry.addData("Pitch 舵机位置 (0–1)", "%.2f", pitch.getPosition());
        else telemetry.addLine("未配置 pitchServo");
        telemetry.addData("Shooter 设定目标 RPM", "%.0f", requestedRpm);
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
