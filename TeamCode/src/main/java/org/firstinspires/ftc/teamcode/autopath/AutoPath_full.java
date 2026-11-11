package org.firstinspires.ftc.teamcode.autopath;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AutoEndPose;
import org.firstinspires.ftc.teamcode.command.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.PathFollowerSubsystem;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath_full extends LinearOpMode {
    private Follower follower;
    private boolean autoStarted;
    private PathFollowerSubsystem drive;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.75, 6.75, 90);
    private final Pose point1 = poseFactory.of(60, 20.7869, 90);
    private final Pose point2 = poseFactory.of(60, 109.75, 90);
    // The visualizer's third segment starts at the same position with a 180-degree heading.
    private final Pose point3Start = poseFactory.of(60, 109.75, 180);
    private final Pose point3 = poseFactory.of(15, 110, 180);

    private Command autoRoutine() {
        return new SequentialCommandGroup(
                new FollowPathCommand(drive, path1()),
                new FollowPathCommand(drive, path2()),
                new FollowPathCommand(drive, path3()));
    }

    @Override
    public void runOpMode() {
        AutoEndPose.clear();
        CommandScheduler.getInstance().reset();
        follower = Constants.create(hardwareMap);
        drive = new PathFollowerSubsystem(follower, this::saveCurrentPose);
        try {
            follower.setPose(start);
            follower.update();

            waitForStart();
            if (isStopRequested()) return;
            autoStarted = true;
            CommandScheduler.getInstance().schedule(autoRoutine());

            while (opModeIsActive()) {
                CommandScheduler.getInstance().run();
                telemetry.addData("x (in)", follower.pose().x());
                telemetry.addData("y (in)", follower.pose().y());
                telemetry.addData("heading (deg)", Math.toDegrees(follower.pose().heading()));
                telemetry.addData("坐标遗传", "当前位置已保存");
                if (follower.currentPath() != null) {
                    telemetry.addData("Distance to endpoint (in)", follower.distanceToEndpoint());
                    telemetry.addData("Path index", follower.pathIndex());
                }
                telemetry.update();
            }
        } finally {
            if (autoStarted) saveCurrentPose();
            CommandScheduler.getInstance().cancelAll();
            drive.stop();
            CommandScheduler.getInstance().reset();
        }
    }

    private void saveCurrentPose() {
        Pose currentPose = follower.pose();
        AutoEndPose.save(currentPose.x(), currentPose.y(), currentPose.heading());
    }

    private Path path1() {
        return Paths.line(start, point1).linear(start, point1);
    }

    private Path path2() {
        return Paths.line(point1, point2).linear(point1, point2);
    }

    private Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }
}
