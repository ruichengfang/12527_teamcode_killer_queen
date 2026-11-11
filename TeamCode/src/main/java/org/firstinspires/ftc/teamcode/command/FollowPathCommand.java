package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.subsystem.PathFollowerSubsystem;

/** 等价于 Ivy 1.1.1 PedroCommands.follow：到参数终点即完成，不额外等待误差收敛。 */
public final class FollowPathCommand extends CommandBase {
    private final PathFollowerSubsystem drive;
    private final Path path;

    public FollowPathCommand(PathFollowerSubsystem drive, Path path) {
        this.drive = drive;
        this.path = path;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.follow(path);
    }

    @Override
    public boolean isFinished() {
        return drive.atParametricEnd();
    }

    @Override
    public void end(boolean interrupted) {
        // 正常完成时不 stop：保留下一段衔接及最后一段的 Pedro 保持行为。
        if (interrupted) drive.stop();
    }
}
