package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NameConstants;

/** 双电机飞轮及其 PD + 前馈转速控制。 */
public class ShooterSubsystem extends SubsystemBase {
    public static final double SINGLE_BALL_RPM = 111.0;

    // 达速判定（移植 OT Shooter.updateReadyState）。KQ 遥测原以“误差 < 200 RPM”作为可发射提示。
    public static final double READY_ENTER_TOLERANCE_RPM = 200.0;
    /** OT 默认退出/进入比 0.025/0.02 = 1.25，换算为 250 RPM。 */
    public static final double READY_EXIT_TOLERANCE_RPM = 250.0;
    /** OT 默认。 */
    public static final double READY_STABLE_SECONDS = 0.05;
    /** OT 默认 30 ticks/s，按 28 ticks/rev 换算为 RPM。 */
    public static final double TARGET_CHANGE_RESET_RPM = 30.0 * 60.0 / 28.0;

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

    private double settleReferenceRpm = Double.NaN;
    private boolean ready;
    private boolean withinTolerance;
    private final ElapsedTime withinToleranceTimer = new ElapsedTime();

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
        setTargetRpm(targetRpm > 0.0 ? 0.0 : SINGLE_BALL_RPM);
    }

    public void start() {
        setTargetRpm(SINGLE_BALL_RPM);
    }

    public void stop() {
        setTargetRpm(0.0);
    }

    /**
     * OT Shooter.setDistanceTarget：距离无效则停机并返回 false。KQ 没有距离-转速表，
     * 有效距离一律使用固定 3300 RPM（等同 OT 的 fixedDistanceTargetVelocity 构造方式）。
     */
    public boolean setDistanceTarget(double distanceCm) {
        if (!Double.isFinite(distanceCm) || distanceCm < 0.0) {
            stop();
            return false;
        }
        setTargetRpm(SINGLE_BALL_RPM);
        return true;
    }

    /** OT Shooter.setTargetVelocity：仅在目标首次设置或累计变化超阈值时重新计时达速。 */
    private void setTargetRpm(double rpm) {
        if (!Double.isFinite(rpm) || rpm <= 0.0) {
            targetRpm = 0.0;
            settleReferenceRpm = Double.NaN;
            resetReadyState();
            return;
        }
        if (targetRpm <= 0.0 || !Double.isFinite(settleReferenceRpm)
                || Math.abs(rpm - settleReferenceRpm) > TARGET_CHANGE_RESET_RPM) {
            settleReferenceRpm = rpm;
            resetReadyState();
        }
        targetRpm = rpm;
    }

    public void update(double deltaSeconds) {
        // shooter0 使用负功率发射，对应的编码器速度取反后再比较正目标转速。
        double rawRpm = -shooter0.getVelocity() * 60.0 / TICKS_PER_REV;
        filteredRpm += RPM_FILTER_ALPHA * (rawRpm - filteredRpm);

        if (targetRpm <= 0.0) {
            power = 0.0;
            error = 0.0;
            lastError = 0.0;
            resetReadyState();
        } else {
            error = targetRpm - filteredRpm;
            double derivative = (error - lastError) / Math.max(deltaSeconds, 1e-4);
            lastError = error;
            power = clamp(KF * targetRpm + KP * error + KD * derivative,
                    MIN_POWER, MAX_POWER);
            updateReadyState(filteredRpm);
        }

        // 保留原程序两台电机的最终输出符号。
        shooter0.setPower(-power);
        shooter1.setPower(power);
    }

    /** OT Shooter.updateReadyState：进入容差需持续稳定，已达速后用更宽的退出容差。 */
    private void updateReadyState(double rpm) {
        double absoluteError = Math.abs(targetRpm - rpm);
        if (!Double.isFinite(absoluteError)) {
            resetReadyState();
            return;
        }
        if (ready) {
            if (absoluteError > READY_EXIT_TOLERANCE_RPM) {
                resetReadyState();
            }
            return;
        }
        if (absoluteError > READY_ENTER_TOLERANCE_RPM) {
            withinTolerance = false;
            return;
        }
        if (!withinTolerance) {
            withinTolerance = true;
            withinToleranceTimer.reset();
            return;
        }
        ready = withinToleranceTimer.seconds() >= READY_STABLE_SECONDS;
    }

    private void resetReadyState() {
        ready = false;
        withinTolerance = false;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public boolean isReady() {
        return ready;
    }
6
    /** KQ 没有俯仰舵机，俯仰条件恒满足。 */
    public boolean isPitchReady() {
        return true;
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
