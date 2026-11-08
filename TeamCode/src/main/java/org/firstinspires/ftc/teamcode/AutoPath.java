package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.75, 6.75, 90);
    private final Pose point1 = poseFactory.of(60, 20.7869, 90);
    private final Pose point2 = poseFactory.of(60, 109.75, 90);
    // The visualizer's third segment starts at the same position with a 180-degree heading.
    private final Pose point3Start = poseFactory.of(60, 109.75, 180);
    private final Pose point3 = poseFactory.of(15, 110, 180);

    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()));
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        try {
            follower.setPose(start);
            follower.update();

            waitForStart();
            if (isStopRequested()) return;
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();
                telemetry.addData("x (in)", follower.pose().x());
                telemetry.addData("y (in)", follower.pose().y());
                telemetry.addData("heading (deg)", Math.toDegrees(follower.pose().heading()));
                if (follower.currentPath() != null) {
                    telemetry.addData("Distance to endpoint (in)", follower.distanceToEndpoint());
                    telemetry.addData("Path index", follower.pathIndex());
                }
                telemetry.update();
            }
        } finally {
            Scheduler.reset();
            follower.stop();
            follower.drivetrain.stop();
        }
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
