package org.firstinspires.ftc.teamcode.shoot;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

/** 移植 OT subsystems/drive/AutoAimController：PID 转向 + 锁定/解锁迟滞。参数见 ShootConstants。 */
public class AutoAimController {
    private final PIDFController controller = new PIDFController(
            ShootConstants.AUTO_AIM_KP,
            ShootConstants.AUTO_AIM_KI,
            ShootConstants.AUTO_AIM_KD,
            0.0
    );

    private boolean locked;
    private boolean withinTolerance;
    private final ElapsedTime withinToleranceTimer = new ElapsedTime();
    private double rawTurnPower;
    private double acquisitionTurnDirection;

    public double calculateTurnPower(boolean hasValidTarget, double yawErrorDegrees,
                                     boolean calibrationReady) {
        if (calibrationReady) {
            if (acquisitionTurnDirection != 0.0) reset();
            return calculateTurnPower(hasValidTarget, yawErrorDegrees);
        }
        double limit = getEffectiveMaxTurnPower();
        double power = ShootConstants.AUTO_AIM_ACQUISITION_TURN_POWER;
        double steeringGain = ShootConstants.AUTO_AIM_KP + ShootConstants.AUTO_AIM_KF;
        double turnSign = ShootConstants.AUTO_AIM_YAW_TO_TURN_SIGN;
        if (!hasValidTarget || !Double.isFinite(yawErrorDegrees)
                || !Double.isFinite(power) || power <= 0.0 || power > 1.0
                || !Double.isFinite(steeringGain) || steeringGain == 0.0
                || !Double.isFinite(turnSign) || turnSign == 0.0 || limit <= 0.0) {
            reset();
            return 0.0;
        }
        locked = false;
        withinTolerance = false;
        if (acquisitionTurnDirection == 0.0) {
            acquisitionTurnDirection = Math.signum(steeringGain)
                    * Math.signum(yawErrorDegrees) * Math.signum(turnSign);
        }
        rawTurnPower = acquisitionTurnDirection * power;
        return clip(rawTurnPower, -limit, limit);
    }

    public double calculateTurnPower(boolean hasValidTarget, double yawErrorDegrees) {
        double maxTurnPower = getEffectiveMaxTurnPower();
        double staticTurnPower = ShootConstants.AUTO_AIM_KS;
        if (!hasValidTarget || !Double.isFinite(yawErrorDegrees)
                || !Double.isFinite(maxTurnPower) || maxTurnPower <= 0.0
                || !Double.isFinite(staticTurnPower) || staticTurnPower < 0.0 || staticTurnPower > 1.0) {
            reset();
            return 0.0;
        }

        controller.setPIDF(
                ShootConstants.AUTO_AIM_KP,
                ShootConstants.AUTO_AIM_KI,
                ShootConstants.AUTO_AIM_KD,
                0.0
        );
        updateLockState(Math.abs(yawErrorDegrees));

        double yawErrorRadians = Math.toRadians(yawErrorDegrees);
        double output = (controller.calculate(-yawErrorRadians, 0.0)
                + ShootConstants.AUTO_AIM_KF * yawErrorRadians)
                * ShootConstants.AUTO_AIM_YAW_TO_TURN_SIGN;
        if (Math.abs(yawErrorDegrees) > ShootConstants.AIM_LOCK_TOLERANCE_DEG) {
            output += Math.signum(output) * staticTurnPower;
        }
        if (!Double.isFinite(output)) {
            reset();
            return 0.0;
        }
        rawTurnPower = output;
        return clip(output, -Math.min(maxTurnPower, 1.0), Math.min(maxTurnPower, 1.0));
    }

    public static double getEffectiveMaxTurnPower() {
        double controllerLimit = ShootConstants.AUTO_AIM_MAX_TURN_POWER;
        double testedLimit = ShootConstants.AUTO_AIM_TEST_MAX_TURN_POWER;
        if (!Double.isFinite(controllerLimit) || !Double.isFinite(testedLimit)
                || controllerLimit < 0.0 || testedLimit < 0.0) {
            return 0.0;
        }
        return Math.min(1.0, Math.min(controllerLimit, testedLimit));
    }

    public double getRawTurnPower() {
        return rawTurnPower;
    }

    public boolean isLocked() {
        return locked;
    }

    public void reset() {
        locked = false;
        withinTolerance = false;
        rawTurnPower = 0.0;
        acquisitionTurnDirection = 0.0;
        controller.reset();
    }

    private void updateLockState(double absoluteYawErrorDegrees) {
        if (locked) {
            if (absoluteYawErrorDegrees > ShootConstants.AIM_UNLOCK_TOLERANCE_DEG) {
                reset();
            }
            return;
        }

        if (absoluteYawErrorDegrees > ShootConstants.AIM_LOCK_TOLERANCE_DEG) {
            withinTolerance = false;
            return;
        }

        if (!withinTolerance) {
            withinTolerance = true;
            withinToleranceTimer.reset();
            return;
        }

        locked = withinToleranceTimer.seconds() >= ShootConstants.AIM_LOCK_STABLE_SECONDS;
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
