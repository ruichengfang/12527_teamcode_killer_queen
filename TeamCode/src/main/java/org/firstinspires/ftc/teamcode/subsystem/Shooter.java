package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NameConstants;

/** 双电机 Shooter：A 开关飞轮；RB/RT/LT 均为按住才进料，松开即停。 */
public class Shooter {

    private static final double TICKS_PER_REV = 28.0; // 电机编码器每圈计数
    private static final double SINGLE_BALL_RPM = 3500.0; // 默认射击转速
    private static final double SHOOTER_KP = 0.0060; // 飞轮比例系数
    private static final double SHOOTER_KD = 0.00001; // 飞轮微分系数
    private static final double SHOOTER_KF = 0.00020; // 飞轮前馈系数
    private static final double SHOOTER_MIN_POWER = -0.1; // 最小飞轮功率
    private static final double SHOOTER_MAX_POWER = 1.0; // 最大飞轮功率
    private static final double INTAKE_SOLO_POWER = 1.0; // RB 单独驱动 intake 的功率
    private static final double TRIGGER_THRESHOLD = 0.05; // 扳机死区

    private final DcMotorEx shooter0;
    private final DcMotorEx shooter1;

    private double targetRPM = 0.0;
    private double filteredRPM = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private double shooterPower = 0.0;
    private double shooterError = 0.0;
    private boolean prevA = false;

    public Shooter(HardwareMap hardwareMap) {
        shooter0 = hardwareMap.get(DcMotorEx.class, NameConstants.SHOOTER_MOTOR_0);
        shooter1 = hardwareMap.get(DcMotorEx.class, NameConstants.SHOOTER_MOTOR_1);

        shooter0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter0.setDirection(DcMotor.Direction.REVERSE);
        shooter1.setDirection(DcMotor.Direction.REVERSE);
    }

    public void start(double now) {
        targetRPM = 0.0;
        filteredRPM = 0.0;
        lastError = 0.0;
        lastTime = now;
        shooterPower = 0.0;
        shooterError = 0.0;
        prevA = false;
        stopMotors();
    }

    /** 遥控每圈调用：A 当帧切换飞轮，进料不设任何定时器。 */
    public void update(Gamepad gamepad, double now,
                       IntakeSubsystem intake, Telemetry telemetry) {
        double dt = now - lastTime;
        if (dt < 1e-4) dt = 1e-4;
        lastTime = now;

        boolean a = gamepad.a;
        if (a && !prevA) {
            targetRPM = targetRPM > 0.0 ? 0.0 : SINGLE_BALL_RPM;
            lastError = 0.0;
        }
        prevA = a;

        // 先处理 A，再计算并输出功率，避免开启后多等一帧。
        updateShooterPD(dt);

        double intakePower = 0.0;
        double transitPower = 0.0;

        // RB：按住时仅 intake 正转；RT/LT：按住时 intake+transit 联动。
        if (gamepad.right_bumper) {
            intakePower = INTAKE_SOLO_POWER;
        } else if (gamepad.right_trigger > TRIGGER_THRESHOLD) {
            intakePower = gamepad.right_trigger;
            transitPower = gamepad.right_trigger;
        } else if (gamepad.left_trigger > TRIGGER_THRESHOLD) {
            intakePower = -gamepad.left_trigger;
            transitPower = -gamepad.left_trigger;
        }

        // IntakeSubsystem 内部会反转 transit 的实际电机方向。
        if (gamepad.right_bumper) {
            intake.setIntakeOnly(intakePower);
        } else {
            intake.setPower(intakePower);
        }

        telemetry.addData("A 输入", a);
        telemetry.addData("Shooter 目标 RPM", "%.0f", targetRPM);
        telemetry.addData("Shooter 实际 RPM", "%.0f", filteredRPM);
        telemetry.addData("Shooter 误差 RPM", "%.0f", shooterError);
        telemetry.addData("Shooter 功率", "%.3f", shooterPower);
        telemetry.addData("Shooter 0/1 输出", "%.2f / %.2f",
                -shooterPower, shooterPower);
        telemetry.addData("Shooter 状态", targetRPM > 0.0 ? "开 (A 关闭)" : "关 (A 开启)");
        telemetry.addData("Intake 功率", "%.2f", intakePower);
        telemetry.addData("Transit 功率", "%.2f", transitPower);
        if (targetRPM > 0.0 && Math.abs(shooterError) > 200.0) {
            telemetry.addLine("Shooter RPM 未稳定，建议误差小于 200 再进料");
        }
    }

    /** 保留给 AutoPath 的转速控制接口，自动供弹由 IntakeSubsystem 完成。 */
    public void update(double runtime,
                       boolean dpadUp, boolean dpadDown, boolean a,
                       boolean rightBumper, boolean b,
                       double rightTrigger, double leftTrigger) {
        double dt = runtime - lastTime;
        if (dt < 1e-4) dt = 1e-4;
        lastTime = runtime;

        if (dpadUp) targetRPM += 1000.0 * dt;
        if (dpadDown) targetRPM -= 1000.0 * dt;
        if (a) targetRPM = 0.0;
        targetRPM = clamp(targetRPM, 0.0, 6000.0);
        updateShooterPD(dt);
    }

    private void updateShooterPD(double dt) {
        double rawRPM = shooter0.getVelocity() * 60.0 / TICKS_PER_REV;
        filteredRPM += 0.4 * (rawRPM - filteredRPM);

        if (targetRPM <= 0.0) {
            shooterPower = 0.0;
            shooterError = 0.0;
            lastError = 0.0;
        } else {
            shooterError = targetRPM - filteredRPM;
            double dError = (shooterError - lastError) / dt;
            lastError = shooterError;
            shooterPower = SHOOTER_KF * targetRPM
                    + SHOOTER_KP * shooterError
                    + SHOOTER_KD * dError;
            shooterPower = clamp(shooterPower, SHOOTER_MIN_POWER, SHOOTER_MAX_POWER);
        }

        shooter0.setPower(-shooterPower);
        shooter1.setPower(shooterPower);
    }

    public void stop() {
        targetRPM = 0.0;
        lastError = 0.0;
        shooterPower = 0.0;
        shooterError = 0.0;
        stopMotors();
    }

    private void stopMotors() {
        shooter0.setPower(0.0);
        shooter1.setPower(0.0);
    }

    public double getTargetRPM() { return targetRPM; }
    public double getFilteredRPM() { return filteredRPM; }
    public double getPower01() { return shooterPower; }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
