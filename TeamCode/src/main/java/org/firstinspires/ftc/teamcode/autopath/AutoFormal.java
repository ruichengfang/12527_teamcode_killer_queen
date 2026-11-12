package org.firstinspires.ftc.teamcode.autopath;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AutoEndPose;
import org.firstinspires.ftc.teamcode.command.FollowPathCommand;
import org.firstinspires.ftc.teamcode.command.ReservedShotWindowCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.PathFollowerSubsystem;

@Autonomous(name = "AutoFormal", group = "Autonomous")
public class AutoFormal extends LinearOpMode {
    private static final long SEGMENT_TIMEOUT_MS = 5000;
    private static final long RESERVED_SHOT_MS = 5000;
    private Follower follower;
    private boolean autoStarted;
    private PathFollowerSubsystem drive;
    private GoBildaPinpointDriver pinpoint;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.2726, 7.9672, 270);
    private final Pose point101path1 = poseFactory.of(57.1091, 21.6998, 270);
    private final Pose point2Start = poseFactory.of(57.1091, 21.6998, 270);
    private final Pose point2 = poseFactory.of(7.4165, 23.4434, 270);
    private final Pose point3 = poseFactory.of(7.0525, 6.141, 270);
    private final Pose point4Start = poseFactory.of(7.0525, 6.141, 270);
    private final Pose point4 = poseFactory.of(57.1357, 121.0297, 270);
    private final Pose point4Control1 = poseFactory.of(34.9821, 80.8158, 0);
    private final Pose point4Control2 = poseFactory.of(38.2982, 96.3828, 0);
    private final Pose point4Control3 = poseFactory.of(16.5318, 84.5853, 0);
    private final Pose point5Start = poseFactory.of(57.1357, 121.0297, 90);
    private final Pose point5 = poseFactory.of(46.9313, 132.8969, 90);
    private final Pose point5Control1 = poseFactory.of(44.6111, 116.5979, 0);
    private final Pose point5Control2 = poseFactory.of(45.8742, 115.0938, 0);
    private final Pose point6 = poseFactory.of(57.2319, 120.5968, 90);
    private final Pose point6Control1 = poseFactory.of(42.3092, 108.1567, 0);
    private final Pose point7Start = poseFactory.of(57.2319, 120.5968, 270);
    private final Pose point7 = poseFactory.of(8.6715, 107.8308, 0);

    private Command autoRoutine() {
        return new SequentialCommandGroup(
                new FollowPathCommand(drive, path101path1(), SEGMENT_TIMEOUT_MS),
                new ReservedShotWindowCommand(drive, RESERVED_SHOT_MS), // Shot point 1: placeholder only
                new FollowPathCommand(drive, path2(), SEGMENT_TIMEOUT_MS),
                new FollowPathCommand(drive, path3(), SEGMENT_TIMEOUT_MS),
                new FollowPathCommand(drive, path4(), SEGMENT_TIMEOUT_MS),
                new ReservedShotWindowCommand(drive, RESERVED_SHOT_MS), // Shot point 4: placeholder only
                new FollowPathCommand(drive, path5(), SEGMENT_TIMEOUT_MS),
                new FollowPathCommand(drive, path6(), SEGMENT_TIMEOUT_MS),
                new ReservedShotWindowCommand(drive, RESERVED_SHOT_MS), // Shot point 6: placeholder only
                new FollowPathCommand(drive, path7(), SEGMENT_TIMEOUT_MS));
    }

    @Override
    public void runOpMode() {
        AutoEndPose.clear();
        CommandScheduler.getInstance().reset();
        if (Constants.foresightConfig == null) {
            telemetry.addLine("Run AutoTune > Foresight Tuner and export its config before running.");
            telemetry.update();
            waitForStart();
            return;
        }
        follower = Constants.create(hardwareMap);
        drive = new PathFollowerSubsystem(follower, this::saveCurrentPose);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        try {
            follower.setPose(start);
            // OT: keep the follower/localizer updating during INIT.
            while (opModeInInit() && !isStopRequested()) {
                follower.update();
                telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            if (!localizationReady()) {
                telemetry.addLine("Pinpoint is not READY; autonomous stopped.");
                telemetry.update();
                return;
            }
            follower.setPose(start);
            autoStarted = true;
            Command routine = autoRoutine();
            CommandScheduler.getInstance().schedule(routine);

            while (opModeIsActive()) {
                // OT: abort the routine if Pinpoint loses READY.
                if (!localizationReady()) {
                    CommandScheduler.getInstance().cancel(routine);
                    drive.stop();
                    telemetry.addLine("Pinpoint lost READY status; autonomous stopped.");
                    telemetry.update();
                    break;
                }
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
            try {
                if (autoStarted) saveCurrentPose();
                CommandScheduler.getInstance().cancelAll();
            } finally {
                try { drive.stop(); }
                finally { CommandScheduler.getInstance().reset(); }
            }
        }
    }

    private boolean localizationReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    private void saveCurrentPose() {
        Pose currentPose = follower.pose();
        AutoEndPose.save(currentPose.x(), currentPose.y(), currentPose.heading());
    }

    private Path path101path1() {
        return Paths.line(start, point101path1).linear(start, point101path1);
    }

    private Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    private Path path3() {
        return Paths.line(point2, point3).constant(point3);
    }

    private Path path4() {
        return Paths.curve(point4Start, point4Control1, point4Control2, point4Control3, point4)
                .linear(point4Start, point4);
    }

    private Path path5() {
        return Paths.curve(point5Start, point5Control1, point5Control2, point5)
                .linear(point5Start, point5);
    }

    private Path path6() {
        return Paths.curve(point5, point6Control1, point6).constant(point6);
    }

    private Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

}
