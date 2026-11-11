package org.firstinspires.ftc.teamcode.shoot;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

/**
 * 移植 OT subsystems/intake/Intake 的射击期 Intake 功率逻辑（排堵 / 射击送料 / 吸入）。
 * KQ 没有颜色距离传感器与花瓣舵机：OT 的计球、满仓反转、传感器失效停转分支不适用，
 * 球数恒为 0，clearBallCount 只保留其“刷新输出”的副作用。
 */
public final class ShotIntake {
    private final IntakeSubsystem intake;
    private double requestedPower;
    private int ballCount;
    private boolean jamRequested;
    private boolean reverseRequested;
    private boolean intakeRequested;
    private boolean shootRequested;
    private boolean shotFeedEnabled = true;

    public ShotIntake(IntakeSubsystem intake) {
        this.intake = intake;
    }

    public IntakeSubsystem getSubsystem() {
        return intake;
    }

    public void updateInputs(boolean intakeRequested, boolean jamRequested) {
        updateInputs(intakeRequested, jamRequested, false);
    }

    public void updateInputs(boolean intakeRequested, boolean jamRequested, boolean shootRequested) {
        if (this.jamRequested && !jamRequested) {
            clearBallCount();
        }
        this.jamRequested = jamRequested;
        this.intakeRequested = intakeRequested;
        this.shootRequested = shootRequested;
        shotFeedEnabled = true;
        updateMotorPower();
    }

    /** 瞄准期间保持已收集的球，只有门打开进入送料后才喂料。 */
    public void updateShotFeed(boolean feeding) {
        jamRequested = false;
        reverseRequested = false;
        intakeRequested = false;
        shootRequested = true;
        shotFeedEnabled = feeding;
        updateMotorPower();
    }

    public void setReverseRequested(boolean reverseRequested) {
        this.reverseRequested = reverseRequested;
        updateMotorPower();
    }

    public void clearBallCount() {
        ballCount = 0;
        updateMotorPower();
    }

    public void suspend() {
        shootRequested = false;
        setPower(0.0);
    }

    public void stop() {
        jamRequested = false;
        reverseRequested = false;
        intakeRequested = false;
        shootRequested = false;
        setPower(0.0);
    }

    public int getBallCount() {
        return ballCount;
    }

    public double getRequestedPower() {
        return requestedPower;
    }

    private void updateMotorPower() {
        if (jamRequested || reverseRequested) {
            setPower(ShootConstants.INTAKE_JAM_POWER);
        } else if (shootRequested) {
            setPower(shotFeedEnabled ? ShootConstants.INTAKE_FEED_POWER : 0.0);
        } else if (intakeRequested) {
            setPower(ShootConstants.INTAKE_ACTIVE_POWER);
        } else {
            setPower(0.0);
        }
    }

    private void setPower(double power) {
        requestedPower = power;
        intake.setIntakePower(requestedPower);
    }
}
