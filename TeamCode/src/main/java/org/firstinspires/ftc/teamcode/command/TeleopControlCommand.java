package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.shoot.AutoAimController;
import org.firstinspires.ftc.teamcode.shoot.PinpointAimTarget;
import org.firstinspires.ftc.teamcode.shoot.ShootConstants;
import org.firstinspires.ftc.teamcode.shoot.TurretAimFeedback;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShovelSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PitchSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 双摇杆驾驶、B 底盘自瞄、飞轮常转、RT 达速开挡板并送料。 */
public final class TeleopControlCommand extends CommandBase {
    private final Gamepad gamepad;
    private final Gamepad operatorGamepad;
    public static final double PINPOINT_TRIM_STEP_IN = 1.0;
    private final PinpointTrimButtons trimButtons = new PinpointTrimButtons();
    private String trimStatus = "未调整";
    private final DoubleSupplier time;
    private final LocalizationSubsystem localization;
    private final TurretSubsystem turret;
    private final BlockerSubsystem blocker;
    private final ShooterSubsystem shooter;
    private final ShovelSubsystem shovel;
    private final PitchSubsystem pitch;
    private final DriveCommand driveCommand;
    private final ShooterCommand shooterCommand;
    private final IntakeCommand intakeCommand;
    private final ChassisAimCommand aimCommand;
    private final PinpointAimTarget aimTarget;
    private final AutoAimController autoAim;
    private final ElapsedTime blockerOpenTimer = new ElapsedTime();
    private final TurretAimFeedback turretFeedback = new TurretAimFeedback();

    private double lastTime, deltaSeconds;
    private boolean previousB;
    private boolean previousX;
    private boolean previousY;
    private boolean previousOperatorLb, previousOperatorRb;
    private boolean previousOperatorY;
    private boolean turretCentered;
    private boolean previousLeftStickButton;
    private boolean previousOptions;
    private boolean previousLeftBumper;
    private boolean aiming;

    public TeleopControlCommand(Gamepad gamepad, Gamepad operatorGamepad, DoubleSupplier time,
            LocalizationSubsystem localization, DriveSubsystem drive,
            IntakeSubsystem intake, ShooterSubsystem shooter,
            TurretSubsystem turret, BlockerSubsystem blocker, ShovelSubsystem shovel, PitchSubsystem pitch) {
        this.gamepad = gamepad;
        this.operatorGamepad = operatorGamepad;
        this.time = time;
        this.localization = localization;
        this.turret = turret;
        this.blocker = blocker;
        this.shooter = shooter;
        this.shovel = shovel;
        this.pitch = pitch;
        driveCommand = new DriveCommand(drive, localization,
                () -> -gamepad.left_stick_y, () -> gamepad.left_stick_x,
                () -> gamepad.right_stick_x);
        shooterCommand = new ShooterCommand(shooter, () -> deltaSeconds);
        intakeCommand = new IntakeCommand(intake, gamepad, () -> blocker.isOpen()
                && blockerOpenTimer.seconds() >= ShooterSubsystem.TRANSIT_DELAY_SECONDS);
        aimTarget = new PinpointAimTarget(localization);
        autoAim = new AutoAimController();
        aimCommand = new ChassisAimCommand(drive, localization, aimTarget, autoAim, gamepad);
        addRequirements(localization, turret, blocker, shovel, pitch);
        for (Command child : new Command[] {driveCommand, shooterCommand, intakeCommand, aimCommand}) {
            addRequirements(child.getRequirements().toArray(new Subsystem[0]));
        }
    }

    @Override
    public void initialize() {
        lastTime = time.getAsDouble();
        previousB = gamepad.b;
        previousX = gamepad.x;
        previousY = gamepad.y;
        previousOperatorLb = operatorGamepad.left_bumper;
        previousOperatorRb = operatorGamepad.right_bumper;
        previousOperatorY = gamepad.y;
        turretCentered = false;
        previousLeftStickButton = gamepad.left_stick_button;
        previousOptions = gamepad.options;
        trimButtons.update(operatorGamepad.dpad_up, operatorGamepad.dpad_down,
                operatorGamepad.dpad_left, operatorGamepad.dpad_right);
        trimStatus = "未调整";
        previousLeftBumper = gamepad.left_bumper;
        aiming = false;
        turretFeedback.reset();
        localization.update();
        localization.resetDriverHeading(); // OT: drive.resetDriverHeading() at START
        blocker.setOpen(false);
        shooter.start();
        driveCommand.initialize();
        shooterCommand.initialize();
        intakeCommand.initialize();
    }

    @Override
    public void execute() {
        double now = time.getAsDouble();
        deltaSeconds = Math.max(now - lastTime, 1e-4);
        lastTime = now;
        localization.update();
        if ("等待 Pinpoint 恢复".equals(trimStatus) && localization.isFieldPoseValid()) {
            trimStatus = "已调整";
        }
        boolean resetPressed = gamepad.options && !previousOptions;
        if (resetPressed) localization.resetToFieldCenter();
        previousOptions = gamepad.options;
        boolean cornerResetPressed = gamepad.y && !previousOperatorY;
        previousOperatorY = gamepad.y;
        if (cornerResetPressed) {
            boolean accepted = localization.resetToNearestCorner();
            trimStatus = !accepted ? "未重置：定位无效" : localization.isFieldPoseValid()
                    ? "已重置到最近角落" : "等待 Pinpoint 恢复";
        }
        trimButtons.update(operatorGamepad.dpad_up, operatorGamepad.dpad_down,
                operatorGamepad.dpad_left, operatorGamepad.dpad_right);
        int dx = trimButtons.getXDirection(), dy = trimButtons.getYDirection();
        if (!resetPressed && !cornerResetPressed && (dx != 0 || dy != 0)) {
            boolean accepted = localization.adjustFieldPosition(
                    dx * PINPOINT_TRIM_STEP_IN, dy * PINPOINT_TRIM_STEP_IN);
            trimStatus = !accepted ? "未调整：定位无效" : localization.isFieldPoseValid()
                    ? "已调整" : "等待 Pinpoint 恢复";
        }
        if (gamepad.x && !previousX) {
            turretCentered = !turretCentered;
            if (turretCentered) turret.setAimMode(false, false);
            blocker.setOpen(false); // 切换射击模式后重新按达速条件开挡板。
        }
        previousX = gamepad.x;
        // 主操 Y 暂时用于 OD 最近角落重置，暂停该键的铲子切换。
        previousY = gamepad.y;
        boolean pitchDown = operatorGamepad.left_bumper;
        boolean pitchUp = operatorGamepad.right_bumper;
        if (pitchDown && !previousOperatorLb && !pitchUp) pitch.adjust(-1);
        if (pitchUp && !previousOperatorRb && !pitchDown) pitch.adjust(1);
        previousOperatorLb = pitchDown;
        previousOperatorRb = pitchUp;
        if (!turretCentered && !turret.isAimMode() && localization.isFieldPoseValid()) {
            turret.setAimMode(true, true);
        }
        turret.update(localization);
        updateTurretFeedback(now);
        aimTarget.update();
        // OT: 左摇杆按下重新设定驾驶员前方。
        if (gamepad.left_stick_button && !previousLeftStickButton) localization.resetDriverHeading();
        previousLeftStickButton = gamepad.left_stick_button;
        // 射击档位：按住 RT 时，LB+RT = 近射，RB+RT = 中射，只按 RT = 远射（两个都按时近射优先）。
        // 松开 RT 时保持上一次的档位（舵机不动）。
        if (turretCentered) {
            if (shooter.getShotMode() != ShooterSubsystem.ShotMode.NEAR)
                shooter.setShotMode(ShooterSubsystem.ShotMode.NEAR);
        } else if (gamepad.right_trigger > ShootConstants.TRIGGER_THRESHOLD) {
            ShooterSubsystem.ShotMode mode = gamepad.left_bumper ? ShooterSubsystem.ShotMode.NEAR
                    : gamepad.right_bumper ? ShooterSubsystem.ShotMode.MID : ShooterSubsystem.ShotMode.FAR;
            if (mode != shooter.getShotMode()) shooter.setShotMode(mode);
        }
        previousLeftBumper = gamepad.left_bumper;
        updateAimButton();
        shooterCommand.execute();
        updateBlocker();
        blocker.flushOutput(); // 每周期写出，Dashboard 改挡板位置立即生效
        intakeCommand.execute();
        if (aiming) {
            aimCommand.execute();
        } else {
            driveCommand.execute();
        }
    }

    /** 无舵机角度反馈：目标在可达范围且指令稳定一段时间后提示一次。 */
    private void updateTurretFeedback(double now) {
        boolean valid = localization.isFieldPoseValid() && turret.isAimMode()
                && turret.hasValidAim() && turret.isReachable();
        if (turretFeedback.update(valid, turret.getCommandedRelativeDegrees(), now)) {
            gamepad.rumble(200);
        }
    }

    public boolean isTurretAimed() {
        return turretFeedback.isAimed();
    }

    public boolean isTurretCentered() { return turretCentered; }

    public boolean isIntakeEnabled() { return intakeCommand.isIntakeEnabled(); }

    public String getTrimStatus() { return trimStatus; }

    /**
     * 按住 RT 且飞轮达速（ShooterSubsystem.isReady，OT 规则，不看自瞄）时打开挡板 blockerServo。
     * 打开后在 RT 松开前保持打开（同 OT：开闸后掉速不中断），松开 RT 立即关闭。
     */
    private void updateBlocker() {
        boolean rtHeld = gamepad.right_trigger > ShootConstants.TRIGGER_THRESHOLD;
        if (!rtHeld) {
            if (blocker.isOpen()) blocker.setOpen(false);
        } else if (!blocker.isOpen() && shooter.isReady()) {
            blocker.setOpen(true);
            blockerOpenTimer.reset();
        }
    }

    private void updateAimButton() {
        boolean pressed = gamepad.b;
        if (pressed && !previousB) {
            aimCommand.initialize();
            aiming = true;
        } else if (!pressed && aiming) {
            aimCommand.end(false);
            aiming = false;
        }
        previousB = pressed;
    }

    @Override
    public void end(boolean interrupted) {
        if (aiming) aimCommand.end(interrupted);
        aiming = false;
        driveCommand.end(interrupted);
        intakeCommand.end(interrupted);
        shooterCommand.end(interrupted);
        blocker.setOpen(false);
    }

    public DriveCommand getDriveCommand() {
        return driveCommand;
    }

    public boolean isAiming() {
        return aiming;
    }

    public PinpointAimTarget getAimTarget() {
        return aimTarget;
    }

    public AutoAimController getAutoAim() {
        return autoAim;
    }
}
