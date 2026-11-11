package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystem.PathFollowerSubsystem;

/** 等价于 Ivy 1.1.1 PedroCommands.follow：到参数终点即完成，不额外等待误差收敛。 */
public final class FollowPathCommand extends CommandBase {
    private final PathFollowerSubsystem drive;
    private final Path path;
    private final long timeoutMs;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean timedOut;

    public FollowPathCommand(PathFollowerSubsystem drive, Path path) {
        this(drive, path, 0);
    }

    public FollowPathCommand(PathFollowerSubsystem drive, Path path, long timeoutMs) {
        this.timeoutMs = timeoutMs;
        this.drive = drive;
        this.path = path;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timedOut = false;
        drive.follow(path);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        if (drive.atParametricEnd()) return true;
        timedOut = timeoutMs > 0 && timer.milliseconds() >= timeoutMs;
        return timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        // 正常完成时不 stop：保留下一段衔接及最后一段的 Pedro 保持行为。
        if (interrupted || timedOut) drive.stop();
    }
}
