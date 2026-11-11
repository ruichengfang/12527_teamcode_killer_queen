package org.firstinspires.ftc.teamcode.shoot;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * 移植 OT subsystems/drive/PositionHoldController：射击时不推左摇杆则锁住进入时的位置。
 * 输出为车体坐标 forward / strafe(右正)，与 KQ DriveSubsystem 约定一致。
 */
public class PositionHoldController {
    private final PIDFController xController = new PIDFController(
            ShootConstants.POSITION_HOLD_KP,
            ShootConstants.POSITION_HOLD_KI,
            ShootConstants.POSITION_HOLD_KD,
            0.0
    );
    private final PIDFController yController = new PIDFController(
            ShootConstants.POSITION_HOLD_KP,
            ShootConstants.POSITION_HOLD_KI,
            ShootConstants.POSITION_HOLD_KD,
            0.0
    );

    private boolean anchored;
    private boolean locked;
    private double anchorX;
    private double anchorY;
    private double forwardPower;
    private double strafePower;
    private double positionErrorInches = Double.NaN;
    private boolean withinTolerance;
    private final ElapsedTime withinToleranceTimer = new ElapsedTime();

    public void update(Pose2D pose, boolean poseValid, double speedInchesPerSecond) {
        forwardPower = 0.0;
        strafePower = 0.0;
        if (!hasValidConfiguration() || pose == null
                || !poseValid
                || !Double.isFinite(speedInchesPerSecond)) {
            reset();
            return;
        }

        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)) {
            reset();
            return;
        }

        if (!anchored) {
            anchorX = x;
            anchorY = y;
            anchored = true;
            positionErrorInches = 0.0;
            xController.reset();
            yController.reset();
            return;
        }

        positionErrorInches = Math.hypot(anchorX - x, anchorY - y);
        updateLockState(positionErrorInches, speedInchesPerSecond);

        if (positionErrorInches <= ShootConstants.POSITION_LOCK_ENTER_TOLERANCE_IN) {
            xController.reset();
            yController.reset();
            return;
        }

        xController.setPIDF(ShootConstants.POSITION_HOLD_KP, ShootConstants.POSITION_HOLD_KI,
                ShootConstants.POSITION_HOLD_KD, 0.0);
        yController.setPIDF(ShootConstants.POSITION_HOLD_KP, ShootConstants.POSITION_HOLD_KI,
                ShootConstants.POSITION_HOLD_KD, 0.0);

        double maxPower = Math.max(0.0, Math.min(1.0, ShootConstants.POSITION_HOLD_MAX_POWER));
        double fieldXPower = clip(xController.calculate(x, anchorX), -maxPower, maxPower);
        double fieldYPower = clip(yController.calculate(y, anchorY), -maxPower, maxPower);
        double magnitude = Math.hypot(fieldXPower, fieldYPower);
        if (magnitude > maxPower && magnitude > 0.0) {
            fieldXPower *= maxPower / magnitude;
            fieldYPower *= maxPower / magnitude;
        }

        forwardPower = fieldXPower * Math.cos(heading) + fieldYPower * Math.sin(heading);
        strafePower = fieldXPower * Math.sin(heading) - fieldYPower * Math.cos(heading);
    }

    public void reset() {
        anchored = false;
        locked = false;
        forwardPower = 0.0;
        strafePower = 0.0;
        positionErrorInches = Double.NaN;
        withinTolerance = false;
        xController.reset();
        yController.reset();
    }

    public boolean isLocked() {
        return locked;
    }

    public double getForwardPower() {
        return forwardPower;
    }

    public double getStrafePower() {
        return strafePower;
    }

    public double getPositionErrorInches() {
        return positionErrorInches;
    }

    private static boolean hasValidConfiguration() {
        return ShootConstants.POSITION_HOLD_CALIBRATED
                && Double.isFinite(ShootConstants.POSITION_HOLD_KP)
                && Double.isFinite(ShootConstants.POSITION_HOLD_KI)
                && Double.isFinite(ShootConstants.POSITION_HOLD_KD)
                && Double.isFinite(ShootConstants.POSITION_HOLD_MAX_POWER)
                && ShootConstants.POSITION_HOLD_MAX_POWER > 0.0
                && ShootConstants.POSITION_HOLD_MAX_POWER <= 1.0
                && Double.isFinite(ShootConstants.POSITION_LOCK_ENTER_TOLERANCE_IN)
                && ShootConstants.POSITION_LOCK_ENTER_TOLERANCE_IN >= 0.0
                && Double.isFinite(ShootConstants.POSITION_LOCK_EXIT_TOLERANCE_IN)
                && ShootConstants.POSITION_LOCK_EXIT_TOLERANCE_IN
                >= ShootConstants.POSITION_LOCK_ENTER_TOLERANCE_IN
                && Double.isFinite(ShootConstants.POSITION_LOCK_MAX_SPEED_IN_PER_SEC)
                && ShootConstants.POSITION_LOCK_MAX_SPEED_IN_PER_SEC >= 0.0
                && Double.isFinite(ShootConstants.POSITION_LOCK_STABLE_SECONDS)
                && ShootConstants.POSITION_LOCK_STABLE_SECONDS >= 0.0;
    }

    private void updateLockState(double errorInches, double speedInchesPerSecond) {
        if (locked) {
            if (errorInches > ShootConstants.POSITION_LOCK_EXIT_TOLERANCE_IN
                    || speedInchesPerSecond > ShootConstants.POSITION_LOCK_MAX_SPEED_IN_PER_SEC) {
                locked = false;
                withinTolerance = false;
            }
            return;
        }

        if (errorInches > ShootConstants.POSITION_LOCK_ENTER_TOLERANCE_IN
                || speedInchesPerSecond > ShootConstants.POSITION_LOCK_MAX_SPEED_IN_PER_SEC) {
            withinTolerance = false;
            return;
        }

        if (!withinTolerance) {
            withinTolerance = true;
            withinToleranceTimer.reset();
            return;
        }

        locked = withinToleranceTimer.seconds() >= ShootConstants.POSITION_LOCK_STABLE_SECONDS;
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
