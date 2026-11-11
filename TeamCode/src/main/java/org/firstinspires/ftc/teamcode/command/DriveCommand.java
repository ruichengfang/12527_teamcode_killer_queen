package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;

/** Gamepad1 双摇杆场地坐标底盘控制。 */
public class DriveCommand extends CommandBase {
    private static final double STICK_DEADZONE = 0.05;

    private final DriveSubsystem drive;
    private final LocalizationSubsystem localization;

    private final DoubleSupplier forward, strafe, turn;

    public DriveCommand(DriveSubsystem drive,
                        LocalizationSubsystem localization,
                        DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        this.drive = drive;
        this.localization = localization;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.driveFieldCentric(applyDeadzone(forward.getAsDouble()), applyDeadzone(strafe.getAsDouble()),
                applyDeadzone(turn.getAsDouble()), localization.getYawRadians());
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }
}
