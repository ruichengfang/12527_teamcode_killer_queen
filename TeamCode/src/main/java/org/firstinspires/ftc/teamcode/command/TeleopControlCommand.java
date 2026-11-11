package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/**
 * 有序的手动控制组合命令。外层独占所有子命令的资源，私有子命令由本组合管理生命周期，
 * 不再单独提交到调度器。这样能保持 jrc 同一周期中的优先级，复位时只取消发射，不中断整个控制循环。
 *
 * 每周期：定位 → B 边沿 → 底盘 → 云台/震动 → 飞轮 → 手动挡板/送料 → 复位 → 联合送料。
 * 云台与飞轮仍每周期更新；B 只接管它们的目标，不会漏掉 PID 或舵机更新。
 */
public final class TeleopControlCommand extends CommandBase {
    private final Gamepad gamepad;
    private final DoubleSupplier time;
    private final LocalizationSubsystem localization;
    private final DriveSubsystem drive;
    private final TurretSubsystem turret;
    private final BlockerSubsystem blocker;
    private final DriveCommand driveCommand;
    private final TurretCommand turretCommand;
    private final ShooterCommand shooterCommand;
    private final BlockerCommand blockerCommand;
    private final IntakeCommand intakeCommand;
    private final ResetLocalizationCommand resetCommand;
    private final AimAndShootCommand aimCommand;

    private double lastTime, deltaSeconds;
    private boolean previousA, previousX, previousY, previousLeft, previousRight;
    private boolean previousLeftStick, previousUp, previousDown, previousB;
    private boolean shooting, releasedB;

    public TeleopControlCommand(Gamepad gamepad, DoubleSupplier time,
            LocalizationSubsystem localization, DriveSubsystem drive,
            IntakeSubsystem intake, ShooterSubsystem shooter,
            TurretSubsystem turret, BlockerSubsystem blocker) {
        this.gamepad = gamepad;
        this.time = time;
        this.localization = localization;
        this.drive = drive;
        this.turret = turret;
        this.blocker = blocker;
        driveCommand = new DriveCommand(drive, localization,
                () -> -gamepad.left_stick_y, () -> gamepad.left_stick_x,
                () -> gamepad.right_stick_x);
        turretCommand = new TurretCommand(turret, localization,
                () -> !shooting && gamepad.x && !previousX,
                () -> !shooting && gamepad.y && !previousY,
                () -> !shooting && gamepad.dpad_left && !previousLeft,
                () -> !shooting && gamepad.dpad_right && !previousRight);
        shooterCommand = new ShooterCommand(shooter,
                () -> !shooting && gamepad.a && !previousA, () -> deltaSeconds);
        blockerCommand = new BlockerCommand(gamepad, blocker,
                () -> !releasedB && gamepad.dpad_up && !previousUp && !gamepad.dpad_down,
                () -> !releasedB && gamepad.dpad_down && !previousDown && !gamepad.dpad_up);
        intakeCommand = new IntakeCommand(intake, gamepad);
        resetCommand = new ResetLocalizationCommand(localization, turret);
        aimCommand = new AimAndShootCommand(shooter, turret, blocker, intake, localization, time);
        for (Command child : new Command[] {driveCommand, turretCommand, shooterCommand,
                blockerCommand, intakeCommand, resetCommand, aimCommand}) {
            addRequirements(child.getRequirements().toArray(new Subsystem[0]));
        }
    }

    @Override
    public void initialize() {
        lastTime = time.getAsDouble();
        previousA = previousX = previousY = previousLeft = previousRight = false;
        previousUp = gamepad.dpad_up;
        previousDown = gamepad.dpad_down;
        previousLeftStick = gamepad.left_stick_button;
        previousB = gamepad.b;
        shooting = releasedB = false;
        driveCommand.initialize();
        turretCommand.initialize();
        shooterCommand.initialize();
        blockerCommand.initialize();
        intakeCommand.initialize();
    }

    @Override
    public void execute() {
        double now = time.getAsDouble();
        deltaSeconds = Math.max(now - lastTime, 1e-4);
        lastTime = now;
        localization.update();
        updateShootingEdge();

        if (shooting) {
            drive.stop();
        } else {
            driveCommand.execute();
        }
        turretCommand.execute();
        if (turret.isAimMode() && !turret.isReachable()) {
            gamepad.rumble(0.8, 0.8, 200);
        } else {
            gamepad.stopRumble();
        }
        shooterCommand.execute();
        if (!shooting) {
            blockerCommand.execute();
            intakeCommand.execute();
        }
        if (gamepad.left_stick_button && !previousLeftStick) {
            // 与 jrc 一致：复位取消 B；保持按住 B 不会再次触发，需松开再按。
            if (shooting) aimCommand.end(true);
            shooting = false;
            resetCommand.initialize();
            resetCommand.end(false);
        }
        rememberButtons();
        if (shooting) {
            blocker.flushOutput();
            aimCommand.execute();
        }
    }

    private void updateShootingEdge() {
        releasedB = !gamepad.b && previousB;
        if (gamepad.b && !previousB) {
            aimCommand.initialize();
            shooting = true;
        } else if (releasedB) {
            if (shooting) aimCommand.end(true);
            shooting = false;
            blocker.setOpen(false);
            blockerCommand.syncButtonState();
        }
    }

    private void rememberButtons() {
        previousA = gamepad.a;
        previousX = gamepad.x;
        previousY = gamepad.y;
        previousLeft = gamepad.dpad_left;
        previousRight = gamepad.dpad_right;
        previousLeftStick = gamepad.left_stick_button;
        previousUp = gamepad.dpad_up;
        previousDown = gamepad.dpad_down;
        previousB = gamepad.b;
    }

    @Override
    public void end(boolean interrupted) {
        if (shooting) aimCommand.end(true);
        shooting = false;
        driveCommand.end(interrupted);
        intakeCommand.end(interrupted);
        shooterCommand.end(interrupted);
        turretCommand.end(interrupted);
        blockerCommand.end(interrupted);
        gamepad.stopRumble();
    }

    public String getShootingState() {
        return aimCommand.getState();
    }
}
