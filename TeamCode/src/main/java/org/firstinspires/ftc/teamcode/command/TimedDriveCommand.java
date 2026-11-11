package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

/** 固定功率直行，结束或取消均归零。 */
public final class TimedDriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final DoubleSupplier time;
    private final double power, durationSeconds;
    private double startedAt;

    public TimedDriveCommand(DriveSubsystem drive, double power, double durationSeconds,
                             DoubleSupplier time) {
        this.drive = drive;
        this.power = power;
        this.durationSeconds = durationSeconds;
        this.time = time;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        startedAt = time.getAsDouble();
        drive.driveFieldCentric(power, 0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return time.getAsDouble() - startedAt >= durationSeconds;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
