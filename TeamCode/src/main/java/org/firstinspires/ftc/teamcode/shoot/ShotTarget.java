package org.firstinspires.ftc.teamcode.shoot;

import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/**
 * 瞄点位置取 KQ 既有值（X=60 in，Y 按车位置在 60/84 in 之间切换，72±1 in 迟滞）；
 * 偏航误差与距离严格按 OT VisionMath.coordinateYawErrorDegrees 计算，
 * 发射口偏角使用 OT 的 shooterYawDegrees = 180°（ShootConstants.SHOOTER_YAW_DEGREES），
 * 不使用 KQ 云台 MOUNT_OFFSET 或旧 AimAndShootCommand 的目标航向约定。
 */
public final class ShotTarget {
    /** OT coordinateYawErrorDegrees 的退化判定：距离 ≤1e-6 in 时方向未定义。 */
    private static final double MIN_DISTANCE_INCHES = 1e-6;

    private double targetXInches = TurretSubsystem.AIM_POINT_X_IN;
    private double targetYInches = TurretSubsystem.AIM_POINT_Y_LOW_IN;
    private double distanceInches = Double.NaN;
    private double desiredHeadingDegrees = Double.NaN;
    private double yawErrorDegrees = Double.NaN;

    public void update(double robotXInches, double robotYInches, double robotHeadingRadians) {
        invalidate();
        if (!Double.isFinite(robotXInches) || !Double.isFinite(robotYInches)
                || !Double.isFinite(robotHeadingRadians)) {
            return;
        }
        // KQ 既有的上下半场瞄点切换。
        if (targetYInches == TurretSubsystem.AIM_POINT_Y_LOW_IN) {
            if (robotYInches > TurretSubsystem.AIM_SWITCH_Y_IN + TurretSubsystem.AIM_SWITCH_HYSTERESIS_IN) {
                targetYInches = TurretSubsystem.AIM_POINT_Y_HIGH_IN;
            }
        } else if (robotYInches < TurretSubsystem.AIM_SWITCH_Y_IN - TurretSubsystem.AIM_SWITCH_HYSTERESIS_IN) {
            targetYInches = TurretSubsystem.AIM_POINT_Y_LOW_IN;
        }
        // OT Vision.updateFieldTarget + VisionMath.coordinateYawErrorDegrees。
        double dx = targetXInches - robotXInches;
        double dy = targetYInches - robotYInches;
        if (Math.hypot(dx, dy) <= MIN_DISTANCE_INCHES) {
            return;
        }
        distanceInches = Math.hypot(dx, dy);
        desiredHeadingDegrees = Math.IEEEremainder(
                Math.toDegrees(Math.atan2(dy, dx)) - ShootConstants.SHOOTER_YAW_DEGREES, 360.0);
        double error = Math.atan2(dy, dx) - Math.toRadians(ShootConstants.SHOOTER_YAW_DEGREES)
                - robotHeadingRadians;
        yawErrorDegrees = Math.toDegrees(Math.atan2(Math.sin(error), Math.cos(error)));
    }

    public void invalidate() {
        distanceInches = Double.NaN;
        desiredHeadingDegrees = Double.NaN;
        yawErrorDegrees = Double.NaN;
    }

    public boolean isValid() {
        return Double.isFinite(yawErrorDegrees) && Double.isFinite(distanceInches);
    }

    public double getTargetXInches() { return targetXInches; }
    public double getTargetYInches() { return targetYInches; }
    public double getDistanceInches() { return distanceInches; }
    public double getDesiredHeadingDegrees() { return desiredHeadingDegrees; }
    public double getYawErrorDegrees() { return yawErrorDegrees; }
}
