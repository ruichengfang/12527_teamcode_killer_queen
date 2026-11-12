package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NameConstants;
import org.firstinspires.ftc.teamcode.tuning.FlywheelTuning;

/** 双电机飞轮，共用自动整定器的 PID + kS + kV 控制器。 */
@Config
public class ShooterSubsystem extends SubsystemBase {
    /** 所有射击档位共用的飞轮目标速度，可在 Dashboard 调整。 */
    public static double SHOT_TICKS_PER_SECOND = 1580.0;
    public static final double SINGLE_BALL_RPM = SHOT_TICKS_PER_SECOND * 60.0 / 28.0;

    // 达速判定照抄 OT ShooterConstants：按目标的比例，不含自瞄条件。
    // 进入 ±2%（2000 TPS 时 ±40 TPS），已达速后放宽到 ±2.5%（±50 TPS）才算掉速；
    // 进入范围后需连续 0.02 s 才算达速。旧值：固定 ±200 / ±250 RPM。
    public static double READY_ENTER_TOLERANCE = 0.02;
    public static double READY_EXIT_TOLERANCE = 0.025;
    public static double READY_STABLE_SECONDS = 0.02;
    /** OT 默认 30 ticks/s，按 28 ticks/rev 换算为 RPM。 */
    public static final double TARGET_CHANGE_RESET_RPM = 30.0 * 60.0 / 28.0;

    private static final double TICKS_PER_REV = 28.0;
    // 2026-11-12 第二次 KQ Shooter Auto PID Test（MAX_POWER 1.0，12.94 V，6 组候选全部达标，复验 score 0.0476）。
    // 平台：0.35→1542 / 0.55→2686 / 0.80→4208 / 1.00→5287 RPM；验证目标 4229 RPM，
    // 覆盖比赛目标 2000 TPS ≈ 4286 RPM。KF 就是 kV，只加一次。
    // 第一次（上限 0.65）：KP 0.000345783 / KI 0.000552456 / KS 0.084462589 / KF 0.000172892。
    // 旧值：KP 0.0060 / KI 0 / KD 0.00001 / KS 0 / KF 0.00020。
    public static double KP = 0.000344490;
    public static double KI = 0.000547963;
    public static double KD = 0.0;
    public static double KS = 0.084063016;
    public static double KF = 0.000172245;
    public static double MAX_POWER = 1.0;
    /** shooterServo 目标位置（0~1），可在 Dashboard 的 ShooterSubsystem 里实时改。实车行程待确认。 */
    // 2026-11-12 用户实车确认：最高点 0.87，最低点 0.91。开机在最低点（远射）。
    public static double SERVO_HIGH = 0.87;
    public static double SERVO_LOW = 0.91;
    public static double SERVO_POSITION = SERVO_LOW;

    /** 射击时（手动 RT / 自动射击）transit 功率；LT 吸球仍为满功率。 */
    public static double SHOT_TRANSIT_POWER = 0.8;
    /** 挡板下达打开指令后留给舵机运动的时间，需实车调整。 */
    public static double TRANSIT_DELAY_SECONDS = 0.2;

    public enum ShotMode { FAR, MID, NEAR }
    private ShotMode shotMode = ShotMode.FAR;

    /** 选档只改变 shooterServo 位置，飞轮使用统一转速。 */
    public void setShotMode(ShotMode mode) {
        shotMode = mode;
        setServoPosition(mode == ShotMode.NEAR ? SERVO_HIGH : mode == ShotMode.MID ? SERVO_HIGH : SERVO_LOW);
        if (targetRpm > 0.0) setTargetRpm(getShotRpm());
    }

    public ShotMode getShotMode() {
        return shotMode;
    }

    public String getShotModeName() {
        return shotMode == ShotMode.NEAR ? "近射" : shotMode == ShotMode.MID ? "中射" : "远射";
    }

    /** 当前档位的目标转速。 */
    public double getShotTicksPerSecond() {
        return SHOT_TICKS_PER_SECOND;
    }

    public double getShotRpm() {
        return getShotTicksPerSecond() * 60.0 / TICKS_PER_REV;
    }

    private final DcMotorEx shooter0;
    private final DcMotorEx shooter1;
    private final Servo shooterServo;

    private double targetRpm;
    private double filteredRpm;
    private FlywheelTuning.Controller controller;
    private String controlFault = "";
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
        shooterServo = hardwareMap.get(Servo.class, NameConstants.SHOOTER_SERVO);
        shooterServo.setPosition(clipServo(SERVO_POSITION));
    }

    public void setServoPosition(double position) {
        SERVO_POSITION = clipServo(position);
        shooterServo.setPosition(SERVO_POSITION);
    }

    /** 在最高点和最低点之间切换；当前不在两者之一时先去最高点。 */
    public void toggleServo() {
        setServoPosition(isServoHigh() ? SERVO_LOW : SERVO_HIGH);
    }

    public boolean isServoHigh() {
        return Math.abs(SERVO_POSITION - SERVO_HIGH) < Math.abs(SERVO_POSITION - SERVO_LOW);
    }

    public double getServoPosition() {
        return SERVO_POSITION;
    }

    private static double clipServo(double position) {
        return Double.isFinite(position) ? Math.max(0.0, Math.min(1.0, position)) : 0.5;
    }

    public void toggle() {
        setTargetRpm(targetRpm > 0.0 ? 0.0 : getShotRpm());
    }

    public void start() {
        setTargetRpm(getShotRpm());
    }

    public void stop() {
        setTargetRpm(0.0);
    }

    /**
     * OT Shooter.setDistanceTarget：距离无效则停机并返回 false。KQ 没有距离-转速表，
     * 有效距离一律使用统一射击转速。
     */
    public boolean setDistanceTarget(double distanceCm) {
        if (!Double.isFinite(distanceCm) || distanceCm < 0.0) {
            stop();
            return false;
        }
        setTargetRpm(getShotRpm());
        return true;
    }

    /** OT Shooter.setTargetVelocity：仅在目标首次设置或累计变化超阈值时重新计时达速。 */
    private void setTargetRpm(double rpm) {
        if (!Double.isFinite(rpm) || rpm <= 0.0) {
            targetRpm = 0.0;
            if (controller != null) controller.reset();
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
        // 只有 shooterMotor1 有编码器（写死）。2026-11-12 实车：原来的取反让转速显示为负数，
        // 闭环把它当成远低于目标，一直加功率。改为取绝对值，与已跑通的 KQ Shooter Auto PID Test
        // 完全相同（参数就是用这个读数测出来的）。shooterMotor2 无编码器，反向同功率跟随。
        double rawRpm = Math.abs(shooter0.getVelocity()) * 60.0 / TICKS_PER_REV;
        try {
            if (!Double.isFinite(rawRpm) || !Double.isFinite(deltaSeconds) || deltaSeconds <= 0)
                throw new IllegalArgumentException("Invalid encoder speed or timestep");
            filteredRpm = FlywheelTuning.filter(filteredRpm, rawRpm, deltaSeconds);
            if (controller == null || controller.p != KP || controller.i != KI || controller.d != KD
                    || controller.ks != KS || controller.kv != KF) {
                controller = new FlywheelTuning.Controller(KP, KI, KD, KS, KF);
                resetReadyState();
            }
            if (targetRpm <= 0.0) {
                power = error = 0.0;
                controller.reset();
                resetReadyState();
            } else {
                error = targetRpm - filteredRpm;
                power = controller.update(targetRpm, filteredRpm, deltaSeconds, MAX_POWER);
                updateReadyState(filteredRpm);
            }
            controlFault = "";
        } catch (IllegalArgumentException invalidInput) {
            controlFault = invalidInput.getMessage();
            power = error = 0.0;
            controller = null;
            resetReadyState();
        }

        // 旧版 PD + F 计算，保留供对照：
        // double derivative = (error - lastError) / Math.max(deltaSeconds, 1e-4);
        // lastError = error;
        // power = Math.max(-0.1, Math.min(1.0,
        //         KF * targetRpm + KP * error + KD * derivative));

        // 保留原程序两台电机的最终输出符号。
        shooter0.setPower(-power);
        shooter1.setPower(power);
        // 每周期写出，Dashboard 改 SERVO_POSITION 立即生效。
        shooterServo.setPosition(clipServo(SERVO_POSITION));
    }

    /** OT Shooter.updateReadyState：进入容差需持续稳定，已达速后用更宽的退出容差。 */
    private void updateReadyState(double rpm) {
        double absoluteError = Math.abs(targetRpm - rpm);
        if (!Double.isFinite(absoluteError)) {
            resetReadyState();
            return;
        }
        if (ready) {
            if (absoluteError > READY_EXIT_TOLERANCE * targetRpm) {
                resetReadyState();
            }
            return;
        }
        if (absoluteError > READY_ENTER_TOLERANCE * targetRpm) {
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

    public String getControlFault() {
        return controlFault;
    }

    public String getEncoderName() {
        return NameConstants.SHOOTER_0;
    }

    public boolean isReady() {
        return ready;
    }

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
