package org.firstinspires.ftc.teamcode.shoot;

import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;

/**
 * 替代 OT 的 Limelight Vision：KQ 没有相机，瞄准目标、偏航误差与距离全部来自 Pinpoint 位姿
 * 和 KQ 既有瞄点。没有视觉校准与视觉纠偏，故“校准就绪”= Pinpoint 位姿有效，
 * wasCorrectionApplied / shouldPauseForAimCalibration 恒为 false。
 * 每周期在 localization.update() 之后调用 update()（对应 OT 调度器中的 vision.updateAimTarget）。
 */
public final class PinpointAimTarget {
    private final LocalizationSubsystem localization;
    private final ShotTarget target = new ShotTarget();
    private boolean calibrationReady;

    public PinpointAimTarget(LocalizationSubsystem localization) {
        this.localization = localization;
    }

    public void update() {
        calibrationReady = localization.isFieldPoseValid();
        if (calibrationReady) {
            target.update(localization.getXIn(), localization.getYIn(),
                    Math.toRadians(localization.getHeadingDeg()));
        } else {
            target.invalidate();
        }
    }

    public boolean isAimCalibrationReady() {
        return calibrationReady;
    }

    public boolean hasValidAimingTarget() {
        return calibrationReady && target.isValid();
    }

    public boolean wasCorrectionApplied() {
        return false;
    }

    public boolean shouldPauseForAimCalibration() {
        return false;
    }

    public double getYawErrorDegrees() {
        return hasValidAimingTarget() ? target.getYawErrorDegrees() : Double.NaN;
    }

    public double getTargetHorizontalDistanceInches() {
        return hasValidAimingTarget() ? target.getDistanceInches() : Double.NaN;
    }

    public double getTargetXInches() { return target.getTargetXInches(); }
    public double getTargetYInches() { return target.getTargetYInches(); }
}
