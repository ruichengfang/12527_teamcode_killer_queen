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
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 双摇杆驾驶、B 底盘自瞄、飞轮常转、RT 达速开挡板并送料。 */
public final class TeleopControlCommand extends CommandBase {
    private final Gamepad gamepad;
    private final DoubleSupplier time;
    private final LocalizationSubsystem localization;
    private final TurretSubsystem turret;
    private final BlockerSubsystem blocker;
    private final ShooterSubsystem shooter;
    private final DriveCommand driveCommand;
    private final ShooterCommand shooterCommand;
    private final IntakeCommand intakeCommand;
    private final ChassisAimCommand aimCommand;
    private final PinpointAimTarget aimTarget;
    private final AutoAimController autoAim;
    private final ElapsedTime blockerOpenTimer = new ElapsedTime();

    private double lastTime, deltaSeconds;
    private boolean previousB;
    private boolean previousLeftStickButton;
    private boolean previousLeftBumper;
    private boolean aiming;

    public TeleopControlCommand(Gamepad gamepad, DoubleSupplier time,
            LocalizationSubsystem localization, DriveSubsystem drive,
            IntakeSubsystem intake, ShooterSubsystem shooter,
            TurretSubsystem turret, BlockerSubsystem blocker) {
        this.gamepad = gamepad;
        this.time = time;
        this.localization = localization;
        this.turret = turret;
        this.blocker = blocker;
        this.shooter = shooter;
        driveCommand = new DriveCommand(drive, localization,
                () -> -gamepad.left_stick_y, () -> gamepad.left_stick_x,
                () -> gamepad.right_stick_x);
        shooterCommand = new ShooterCommand(shooter, () -> true, () -> deltaSeconds);
        intakeCommand = new IntakeCommand(intake, gamepad, () -> blocker.isOpen()
                && blockerOpenTimer.seconds() >= ShooterSubsystem.TRANSIT_DELAY_SECONDS);
        aimTarget = new PinpointAimTarget(localization);
        autoAim = new AutoAimController();
        aimCommand = new ChassisAimCommand(drive, localization, aimTarget, autoAim, gamepad);
        addRequirements(localization, turret, blocker);
        for (Command child : new Command[] {driveCommand, shooterCommand, intakeCommand, aimCommand}) {
            addRequirements(child.getRequirements().toArray(new Subsystem[0]));
        }
    }

    @Override
    public void initialize() {
        lastTime = time.getAsDouble();
        previousB = gamepad.b;
        previousLeftStickButton = gamepad.left_stick_button;
        previousLeftBumper = gamepad.left_bumper;
        aiming = false;
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
        turret.update(localization);
        aimTarget.update();
        // OT: 左摇杆按下重新设定驾驶员前方。
        if (gamepad.left_stick_button && !previousLeftStickButton) localization.resetDriverHeading();
        previousLeftStickButton = gamepad.left_stick_button;
        // 射击档位：按住 RT 时，LB+RT = 近射，RB+RT = 中射，只按 RT = 远射（两个都按时近射优先）。
        // 松开 RT 时保持上一次的档位（舵机不动）。
        if (gamepad.right_trigger > ShootConstants.TRIGGER_THRESHOLD) {
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
