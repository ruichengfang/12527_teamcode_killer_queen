package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

/** 自动模式唯一的 Pedro 底盘所有者，不与手动 DriveSubsystem 同时实例化。 */
public final class PathFollowerSubsystem extends SubsystemBase {
    private final Follower follower;
    private final Runnable afterUpdate;

    public PathFollowerSubsystem(Follower follower) {
        this(follower, () -> {});
    }

    /** 更新定位后、调度路径命令前保存末位置，顺序与 jrc 一致。 */
    public PathFollowerSubsystem(Follower follower, Runnable afterUpdate) {
        this.follower = follower;
        this.afterUpdate = afterUpdate;
    }

    @Override
    public void periodic() {
        follower.update();
        afterUpdate.run();
    }

    public void follow(Path path) {
        follower.follow(path);
    }

    public boolean atParametricEnd() {
        return follower.atParametricEnd();
    }

    public void stop() {
        follower.stop();
        follower.drivetrain.stop();
    }
}
