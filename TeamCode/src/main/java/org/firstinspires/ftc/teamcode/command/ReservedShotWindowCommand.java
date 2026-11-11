package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystem.PathFollowerSubsystem;

/** Reserved shooting time only: stop the chassis; do not start shooter/intake/transit. */
public final class ReservedShotWindowCommand extends CommandBase {
    private final PathFollowerSubsystem drive;
    private final long durationMs;
    private final ElapsedTime timer = new ElapsedTime();
    public ReservedShotWindowCommand(PathFollowerSubsystem drive, long durationMs) {
        if (durationMs <= 0) throw new IllegalArgumentException("Positive finite shot window required");
        this.drive = drive;
        this.durationMs = durationMs;
        addRequirements(drive);
    }
    @Override public void initialize() { drive.stop(); timer.reset(); }
    @Override public void execute() { drive.stop(); }
    @Override public boolean isFinished() { return timer.milliseconds() >= durationMs; }
    @Override public void end(boolean interrupted) { drive.stop(); }
}
