package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystem.PathFollowerSubsystem;

/**
 * OT 自动 followPath：follow 后等到 Pedro !isBusy() 才完成；超时（>0）则停车并进入下一段。
 * 旧版（Ivy 等价）是到参数终点即换段，已改为 OT 行为。
 */
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
        if (!drive.isBusy()) return true;
        timedOut = timeoutMs > 0 && timer.milliseconds() >= timeoutMs;
        return timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        // 正常完成时不 stop：保留下一段衔接及最后一段的 Pedro 保持行为。
        if (interrupted || timedOut) drive.stop();
    }
}
