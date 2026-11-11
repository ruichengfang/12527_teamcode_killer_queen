package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.shoot.ShotIntake;
import org.firstinspires.ftc.teamcode.shoot.ShotTarget;
import org.firstinspires.ftc.teamcode.shoot.ShotTransit;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

import java.util.function.BooleanSupplier;

/**
 * 移植 Overtune commands/AutoShootCommand：停下 follower，用 Pedro 场地位姿做一次距离射击，
 * 送料满 feedDurationMs 或超时后自行结束。OT 的 AutoHiveTarget（视觉）由 KQ Pinpoint 瞄点
 * ShotTarget 替代（偏航误差仅作遥测，自动射击本身不转向，与 OT 相同）。
 * KQ 飞轮没有 periodic，本命令每周期自行调用 ShooterSubsystem.update()（对应 OT Shooter.periodic）。
 * 按用户要求，当前没有任何自动程序调用本命令。
 */
public class AutoShootCommand extends CommandBase {
    private static final long PRE_SHOT_DELAY_MS = 7_000;

    private final Follower follower;
    private final ShooterSubsystem shooter;
    private final ShotIntake intake;
    private final ShotTransit transit;
    private final ShotTarget target;
    private final BooleanSupplier localizationReady;
    private final long feedDurationMs;
    private final long timeoutMs;
    private final boolean delayBeforeShooting;

    private final ElapsedTime startedTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean feedStarted;
    private boolean waitingForDelay;
    private boolean timedOut;
    private String status = "Not started";

    public AutoShootCommand(Follower follower, ShooterSubsystem shooter, ShotIntake intake,
                            ShotTransit transit, ShotTarget target, BooleanSupplier localizationReady,
                            long feedDurationMs, long timeoutMs, boolean delayBeforeShooting) {
        if (feedDurationMs <= 0 || timeoutMs < feedDurationMs) {
            throw new IllegalArgumentException("Shot timeout must cover a positive feed duration");
        }
        this.follower = follower;
        this.shooter = shooter;
        this.intake = intake;
        this.transit = transit;
        this.target = target;
        this.localizationReady = localizationReady;
        this.feedDurationMs = feedDurationMs;
        this.timeoutMs = timeoutMs;
        this.delayBeforeShooting = delayBeforeShooting;
        addRequirements(shooter, intake.getSubsystem(), transit.getBlocker());
    }

    @Override
    public void initialize() {
        startedTimer.reset();
        loopTimer.reset();
        feedStarted = false;
        waitingForDelay = delayBeforeShooting;
        timedOut = false;
        status = "Preparing shot";
        follower.stop();
        follower.drivetrain.stop();
        transit.cancelFeeding();
        intake.updateShotFeed(false);
        // 保留前一段路径中已设定的飞轮目标。
    }

    @Override
    public void execute() {
        // OT Shooter.periodic 先于命令执行：先用上一周期目标更新 PID 与达速判定。
        shooter.update(Math.max(loopTimer.seconds(), 1e-4));
        loopTimer.reset();
        if (isFinished()) return;
        if (waitingForDelay && startedTimer.milliseconds() >= PRE_SHOT_DELAY_MS) {
            waitingForDelay = false;
            startedTimer.reset();
        }

        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        if (!localizationReady.getAsBoolean() || pose == null || velocity == null
                || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading()) || !Double.isFinite(velocity.linSpeed())
                || !Double.isFinite(velocity.omega)) {
            stopForInvalidTarget("Waiting for valid localization");
            return;
        }

        target.update(pose.x(), pose.y(), pose.heading());
        double distanceCm = target.isValid()
                ? DistanceUnit.INCH.toCm(target.getDistanceInches()) : Double.NaN;
        if (!shooter.setDistanceTarget(distanceCm)) {
            stopForInvalidTarget("Waiting for a valid shot target");
            return;
        }

        if (waitingForDelay) {
            status = "Waiting " + PRE_SHOT_DELAY_MS + " ms before shot";
            return;
        }

        boolean speedReady = shooter.isReady();
        transit.requestFeed(speedReady);
        ShotTransit.FiringState firingState = transit.getFiringState();
        boolean feeding = firingState == ShotTransit.FiringState.FEEDING;
        intake.updateShotFeed(feeding);
        if (feeding) {
            if (!feedStarted) {
                feedStarted = true;
                feedTimer.reset();
            }
            status = "Shooting: " + feedDurationMs + " ms";
        } else if (firingState == ShotTransit.FiringState.REVERSING) {
            status = "Reversing before feed";
        } else if (!speedReady) {
            status = "Waiting for shooter speed";
        } else {
            status = "Invalid transit gate or reverse settings";
        }
    }

    @Override
    public boolean isFinished() {
        if (feedStarted) {
            return feedTimer.milliseconds() >= feedDurationMs;
        }
        timedOut = !waitingForDelay && startedTimer.milliseconds() >= timeoutMs;
        return timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        if (transit.getState() == ShotTransit.TransitState.OPEN) intake.clearBallCount();
        transit.cancelFeeding();
        intake.stop();
        shooter.stop();
        shooter.update(1e-4); // KQ 飞轮无 periodic，立即写出零功率。
        follower.stop();
        follower.drivetrain.stop();
        if (interrupted) {
            status = "Shot interrupted";
        } else if (timedOut) {
            status = "Shot timed out (" + status + ")";
        } else {
            status = "Shot complete";
        }
    }

    public String getStatus() {
        return status;
    }

    public boolean hasTimedOut() {
        return timedOut;
    }

    private void stopForInvalidTarget(String reason) {
        if (transit.getState() == ShotTransit.TransitState.OPEN) intake.clearBallCount();
        transit.cancelFeeding();
        intake.updateShotFeed(false);
        feedStarted = false;
        status = reason;
        shooter.stop();
        follower.stop();
        follower.drivetrain.stop();
    }
}
