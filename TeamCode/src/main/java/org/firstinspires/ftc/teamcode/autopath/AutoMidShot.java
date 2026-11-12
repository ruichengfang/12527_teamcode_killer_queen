package org.firstinspires.ftc.teamcode.autopath;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.shoot.ShootConstants;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AutoEndPose;
import org.firstinspires.ftc.teamcode.command.AutoSoloShotCommand;
import org.firstinspires.ftc.teamcode.command.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PathFollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/**
 * 2026-11-12 新自动（4 段）：只在第 1 段终点打一次中射（与 Solo RB+RT 相同）。
 * 每段路径 5 秒超时、每次射击 5 秒超时，超时直接进入下一步。
 */
@Autonomous(name = "AutoMidShot", group = "Autonomous")
public class AutoMidShot extends LinearOpMode {
    private static final long SEGMENT_TIMEOUT_MS = 5000;
    private Follower follower;
    private boolean autoStarted;
    private PathFollowerSubsystem drive;
    private GoBildaPinpointDriver pinpoint;
    private int pinpointFaults;
    private String lastFault = "";
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private BlockerSubsystem blocker;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.2726, 7.9672, 270);
    private final Pose point101path101path1 = poseFactory.of(59.7908, 10, 270);
    private final Pose point2 = poseFactory.of(23.3669, 21.5333, 270);
    private final Pose point3 = poseFactory.of(23.8706, 107.6622, 270);
    private final Pose point407path7Start = poseFactory.of(23.8706, 107.6622, 0);
    private final Pose point407path7 = poseFactory.of(8.6715, 107.8308, 0);

    private Command midShot() {
        return new AutoSoloShotCommand(drive, shooter, intake, blocker, ShooterSubsystem.ShotMode.MID);
    }

    private Command autoRoutine() {
        return new SequentialCommandGroup(
                new FollowPathCommand(drive, path101path101path1(), SEGMENT_TIMEOUT_MS),
                midShot(),
                new FollowPathCommand(drive, path2(), SEGMENT_TIMEOUT_MS),
                new FollowPathCommand(drive, path3(), SEGMENT_TIMEOUT_MS),
                new FollowPathCommand(drive, path407path7(), SEGMENT_TIMEOUT_MS));
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
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        blocker = new BlockerSubsystem(hardwareMap);
        new TurretSubsystem(hardwareMap, true); // 云台固定中位，与 Solo 相同
        try {
            follower.setPose(start);
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
                // 2026-11-12：原来 Pinpoint 只要一帧不是 READY（吸球/送料电机全开时 I2C 常见的
                // FAULT_BAD_READ）就 break 结束整个自动。现在只暂停：底盘停住、继续读 Pinpoint，
                // 恢复 READY 后接着当前步骤跑；不再结束自动。
                if (!localizationReady()) {
                    drive.stop();
                    follower.update();
                    pinpointFaults++;
                    lastFault = String.valueOf(pinpoint.getDeviceStatus());
                    telemetry.addData("Pinpoint 暂时异常，暂停等待恢复", "%s（累计 %d 次）", lastFault, pinpointFaults);
                    telemetry.update();
                    idle();
                    continue;
                }
                CommandScheduler.getInstance().run();
                if (pinpointFaults > 0) telemetry.addData("Pinpoint 异常次数 / 最近状态", "%d / %s", pinpointFaults, lastFault);
                telemetry.addData("x / y (in)", "%.1f / %.1f", follower.pose().x(), follower.pose().y());
                telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
                telemetry.addData("射击档位", "%s / 目标 %.0f TPS / 实际 %.0f RPM / 达速 %s",
                        shooter.getShotModeName(), shooter.getShotTicksPerSecond(),
                        shooter.getFilteredRpm(), shooter.isReady() ? "是" : "否");
                telemetry.addData("挡板", blocker.isOpen() ? "开" : "关");
                telemetry.addData("intake / transit 功率", "%.2f / %.2f",
                        intake.getIntakePower(), intake.getTransitPower());
                telemetry.update();
            }
        } finally {
            try {
                if (autoStarted) saveCurrentPose();
                CommandScheduler.getInstance().cancelAll();
            } finally {
                try {
                    drive.stop();
                    blocker.setOpen(false);
                    intake.stop();
                    shooter.stop();
                    shooter.update(1e-4);
                } finally { CommandScheduler.getInstance().reset(); }
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

    public Path path101path101path1() {
        return Paths.line(start, point101path101path1).linear(start, point101path101path1);
    }

    public Path path2() {
        return Paths.line(point101path101path1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).constant(point3);
    }

    public Path path407path7() {
        return Paths.line(point407path7Start, point407path7).linear(point407path7Start, point407path7);
    }
}
