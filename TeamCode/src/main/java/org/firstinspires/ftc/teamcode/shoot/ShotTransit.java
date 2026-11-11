package org.firstinspires.ftc.teamcode.shoot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

/**
 * 移植 OT subsystems/transit/Transit 的射击送料状态机。KQ 的 transitMotor 归 IntakeSubsystem，
 * 门（OT limitServo）为 KQ blockerServo（BlockerSubsystem），因此本类不是独立子系统，
 * 调用它的命令须占用 intake 与 blocker 两个子系统。手动进料仍由 KQ 原 IntakeCommand 负责。
 */
public final class ShotTransit {
    public enum FiringState {
        IDLE,
        WAITING,
        REVERSING,
        FEEDING
    }

    public enum TransitState {
        CLOSE,
        OPEN
    }

    private final IntakeSubsystem intake;
    private final BlockerSubsystem blocker;
    private TransitState transitState = TransitState.CLOSE;
    private double requestedPower;
    private final ElapsedTime feedReverseTimer = new ElapsedTime();
    private double feedReverseTimeSeconds;
    private double feedReversePower;
    private FiringState firingState = FiringState.IDLE;

    public ShotTransit(IntakeSubsystem intake, BlockerSubsystem blocker) {
        this.intake = intake;
        this.blocker = blocker;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intake;
    }

    public BlockerSubsystem getBlocker() {
        return blocker;
    }

    /** 门一旦打开，只有显式取消才会结束“反转 → 送料”序列。 */
    public void requestFeed(boolean readyToOpenLimit) {
        if (!isFeedActive()) {
            double reverseTime = ShootConstants.FEED_REVERSE_TIME_SECONDS;
            double reversePower = ShootConstants.FEED_REVERSE_POWER;
            if (!readyToOpenLimit || !areLimitPositionsCalibrated()
                    || !Double.isFinite(reverseTime) || reverseTime < 0.0
                    || !Double.isFinite(reversePower) || reversePower >= 0.0 || reversePower < -1.0) {
                cancelFeeding();
                firingState = FiringState.WAITING;
                return;
            }
            stopMotor();
            open();
            feedReverseTimeSeconds = reverseTime;
            feedReversePower = reversePower;
            feedReverseTimer.reset();
            firingState = FiringState.REVERSING;
        }
        if (firingState == FiringState.REVERSING) {
            if (feedReverseTimer.seconds() >= feedReverseTimeSeconds) {
                firingState = FiringState.FEEDING;
            } else {
                setPower(feedReversePower);
            }
        }
        if (firingState == FiringState.FEEDING) {
            setPower(ShootConstants.TRANSIT_FEED_POWER);
        }
    }

    public boolean isFeedActive() {
        return firingState == FiringState.REVERSING || firingState == FiringState.FEEDING;
    }

    public void cancelFeeding() {
        stopMotor();
        close();
        firingState = FiringState.IDLE;
        feedReverseTimer.reset();
    }

    public FiringState getFiringState() {
        return firingState;
    }

    public void forward() {
        setPower(ShootConstants.TRANSIT_FORWARD_POWER);
    }

    public void forwardForIntakeReverse() {
        setPower(ShootConstants.TRANSIT_INTAKE_REVERSE_FORWARD_POWER);
    }

    public void reverse() {
        setPower(ShootConstants.TRANSIT_REVERSE_POWER);
    }

    public void stopMotor() {
        setPower(0.0);
    }

    public double getRequestedPower() {
        return requestedPower;
    }

    public boolean areLimitPositionsCalibrated() {
        return ShootConstants.LIMIT_POSITIONS_CALIBRATED
                && validServoPosition(ShootConstants.LIMIT_CLOSED)
                && validServoPosition(ShootConstants.LIMIT_OPEN)
                && ShootConstants.LIMIT_CLOSED != ShootConstants.LIMIT_OPEN;
    }

    public void close() {
        setState(TransitState.CLOSE);
    }

    public void open() {
        if (!areLimitPositionsCalibrated()) {
            close();
            return;
        }
        setState(TransitState.OPEN);
    }

    public TransitState getState() {
        return transitState;
    }

    private void setState(TransitState state) {
        transitState = state;
        if (areLimitPositionsCalibrated()) {
            blocker.setOpen(state == TransitState.OPEN);
        }
    }

    private void setPower(double power) {
        requestedPower = Double.isFinite(power) ? Math.max(-1.0, Math.min(1.0, power)) : 0.0;
        intake.setTransitPower(requestedPower);
    }

    private static boolean validServoPosition(double position) {
        return Double.isFinite(position) && position >= 0.0 && position <= 1.0;
    }
}
