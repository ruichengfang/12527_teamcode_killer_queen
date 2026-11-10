package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NameConstants;

/** 双电机飞轮及其 PD + 前馈转速控制。 */
public class ShooterSubsystem extends SubsystemBase {
    public static final double SINGLE_BALL_RPM = 3300.0;

    private static final double TICKS_PER_REV = 28.0;
    private static final double KP = 0.0060;
    private static final double KD = 0.00001;
    private static final double KF = 0.00020;
    private static final double MIN_POWER = -0.1;
    private static final double MAX_POWER = 1.0;
    private static final double RPM_FILTER_ALPHA = 0.4;

    private final DcMotorEx shooter0;
    private final DcMotorEx shooter1;

    private double targetRpm;
    private double filteredRpm;
    private double lastError;
    private double power;
    private double error;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooter0 = hardwareMap.get(DcMotorEx.class, NameConstants.SHOOTER_0);
        shooter1 = hardwareMap.get(DcMotorEx.class, NameConstants.SHOOTER_1);

        shooter0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter0.setDirection(DcMotor.Direction.REVERSE);
        shooter1.setDirection(DcMotor.Direction.REVERSE);
    }

    public void toggle() {
        targetRpm = targetRpm > 0.0 ? 0.0 : SINGLE_BALL_RPM;
    }

    public void start() {
        targetRpm = SINGLE_BALL_RPM;
    }

    public void stop() {
        targetRpm = 0.0;
    }

    public void update(double deltaSeconds) {
        double rawRpm = shooter0.getVelocity() * 60.0 / TICKS_PER_REV;
        filteredRpm += RPM_FILTER_ALPHA * (rawRpm - filteredRpm);

        if (targetRpm <= 0.0) {
            power = 0.0;
            error = 0.0;
            lastError = 0.0;
        } else {
            error = targetRpm - filteredRpm;
            double derivative = (error - lastError) / Math.max(deltaSeconds, 1e-4);
            lastError = error;
            power = clamp(KF * targetRpm + KP * error + KD * derivative,
                    MIN_POWER, MAX_POWER);
        }

        // 保留原程序两台电机的最终输出符号。
        shooter0.setPower(-power);
        shooter1.setPower(power);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getFilteredRpm() {
        return filteredRpm;
    }

    public double getError() {
        return error;
    }

    public double getPower() {
        return power;
    }

    public boolean isRunning() {
        return targetRpm > 0.0;
    }
}
