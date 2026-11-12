package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.NameConstants;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;
import org.firstinspires.ftc.teamcode.tuning.FlywheelTuning;

/**
 * Open-loop identification followed by measured, bounded PID candidate trials.
 * Feedback is hard-wired: only shooterMotor1 (SHOOTER_0) has an encoder. shooterMotor2 has none
 * and simply receives the same power magnitude with the opposite sign (the two motors spin in
 * opposite directions), exactly like ShooterSubsystem's output.
 */
@Config
@TeleOp(name = "KQ Shooter Auto PID Test", group = "Test")
public final class ShooterAutoTune extends LinearOpMode {
    public static double MAX_POWER = .65;
    public static double MAX_RPM = 6500;
    public static double TARGET_RPM = 0; // 0 chooses 80% of measured top plateau.
    public static double TICKS_PER_REV = 28;
    private DcMotorEx leader, follower; // leader = shooterMotor1 (encoder), follower = shooterMotor2
    private double maxPower, maxRpm, targetRequest, ticksPerRev;
    private double started, lastSample, filteredRpm, initialVoltage, commandedPower;
    private double lastDisplay;
    private double requestedRpm;
    private double validatedTargetRpm;
    private double coastRemaining;
    private FlywheelTuning.Controller result;
    private String resultFile = "";
    private double lowSpeedStarted = Double.NaN;
    private String phase = "INIT";
    private boolean lastQualified;
    private final StringBuilder warnings = new StringBuilder();
    private final StringBuilder report = new StringBuilder();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        String outcome = "未完成";
        try {
            leader = hardwareMap.get(DcMotorEx.class, NameConstants.SHOOTER_0);
            follower = hardwareMap.get(DcMotorEx.class, NameConstants.SHOOTER_1);
            for (DcMotorEx motor : new DcMotorEx[] {leader, follower}) {
                motor.setPower(0);
                motor.setDirection(DcMotor.Direction.REVERSE);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            new IntakeSubsystem(hardwareMap).stop();
            new BlockerSubsystem(hardwareMap).setOpen(false);
            new TurretSubsystem(hardwareMap, true);
            for (String name : new String[] {NameConstants.FRONT_LEFT, NameConstants.FRONT_RIGHT,
                    NameConstants.BACK_LEFT, NameConstants.BACK_RIGHT}) {
                hardwareMap.get(DcMotorEx.class, name).setPower(0);
            }
            while (!isStarted() && !isStopRequested()) {
                telemetry.addLine("清空球。Dashboard 点 Start 开始，点 Stop 停止，无需手柄。");
                telemetry.addLine("转速只读 shooterMotor1 编码器；shooterMotor2 反向同功率跟随。");
                publishLive("已初始化，等待 Dashboard Start");
                sleep(200);
            }
            waitForStart();
            if (isStopRequested()) return;
            maxPower = MAX_POWER; maxRpm = MAX_RPM;
            targetRequest = TARGET_RPM; ticksPerRev = TICKS_PER_REV;
            // Never refuse to run: fall back to defaults for any invalid Dashboard value.
            if (!Double.isFinite(maxPower) || maxPower < .3 || maxPower > 1) { warn("MAX_POWER 无效，按 0.65"); maxPower = .65; }
            if (!Double.isFinite(maxRpm) || maxRpm < 100) { warn("MAX_RPM 无效，按 6500"); maxRpm = 6500; }
            if (!Double.isFinite(ticksPerRev) || ticksPerRev <= 0) { warn("TICKS_PER_REV 无效，按 28"); ticksPerRev = 28; }
            if (!Double.isFinite(targetRequest) || targetRequest < 0) { warn("TARGET_RPM 无效，按自动"); targetRequest = 0; }
            started = lastSample = now();
            initialVoltage = voltage();
            report.append(String.format(Locale.US, "Battery %.2f V; %.2f ticks/rev\n",
                    initialVoltage, ticksPerRev));
            report.append("使用编码器：").append(NameConstants.SHOOTER_0)
                    .append("（固定）；").append(NameConstants.SHOOTER_1).append(" 反向同功率跟随\n");
            runCalibration();
            outcome = warnings.length() == 0 ? "完成，参数及验证结果见下方" : "完成（有警告，参数需人工确认），见下方";
        } catch (RuntimeException error) {
            outcome = "中止：" + error.getMessage();
        } catch (InterruptedException interrupted) {
            outcome = "已由 Stop 中止";
            throw interrupted;
        } finally {
            if (leader != null) leader.setPower(0);
            if (follower != null) follower.setPower(0);
            report.append("\n").append(outcome).append("\n");
            RobotLog.ii("KQAutoTune", "%s", report.toString());
            commandedPower = 0;
            resultFile = saveReport();
            publishResult(outcome);
        }
        while (opModeIsActive()) {
            publishResult(outcome);
            sleep(250);
        }
    }

    private void runCalibration() throws InterruptedException {
        phase = "等待飞轮静止";
        double coastStart = now();
        while (now() - coastStart < 10) {
            coastRemaining = Math.max(0, 10 - (now() - coastStart));
            setPower(0);
            sample();
            // Already still at START: continue after 0.3 s of readings instead of idling 2 s.
            if (now() - coastStart > .3 && Math.abs(filteredRpm) < 50) break;
        }
        coastRemaining = 0;
        if (Math.abs(filteredRpm) > 50) warn(String.format(Locale.US, "开始时飞轮没停稳（%.0f RPM），照常继续", filteredRpm));
        double[] powers = {maxPower * .35, maxPower * .55, maxPower * .8, maxPower};
        double[] speeds = new double[powers.length];
        List<double[]> firstStep = new ArrayList<>();
        for (int stage = 0; stage < powers.length; stage++) {
            phase = "测前馈 " + (stage + 1) + "/4";
            double start = now(), sum = 0, sumSquares = 0, firstHalf = 0, secondHalf = 0;
            int count = 0, firstCount = 0, secondCount = 0;
            setPower(powers[stage]);
            while (now() - start < 5) {
                sample();
                double elapsed = now() - start;
                if (stage == 0) firstStep.add(new double[] {elapsed, filteredRpm});
                if (elapsed >= 4) {
                    sum += filteredRpm; sumSquares += filteredRpm * filteredRpm; count++;
                    if (elapsed < 4.5) { firstHalf += filteredRpm; firstCount++; }
                    else { secondHalf += filteredRpm; secondCount++; }
                }
            }
            if (count < 5) warn(String.format(Locale.US, "功率 %.2f 只采到 %d 个点，控制循环偏慢", powers[stage], count));
            speeds[stage] = count > 0 ? sum / count : filteredRpm;
            double deviation = count > 0 ? Math.sqrt(Math.max(0, sumSquares / count - speeds[stage] * speeds[stage])) : 0;
            if (firstCount == 0 || secondCount == 0) { firstHalf = secondHalf = speeds[stage]; firstCount = secondCount = 1; }
            // OT tolerance: max(30, 6%) instead of the stricter 4%/5%.
            double band = Math.max(30, .06 * speeds[stage]);
            if (deviation > band || Math.abs(secondHalf / secondCount - firstHalf / firstCount) > band)
                warn(String.format(Locale.US, "功率 %.2f 时转速不够稳（%.0f±%.0f RPM），仍继续", powers[stage], speeds[stage], deviation));
            if (stage > 0 && speeds[stage] <= speeds[stage - 1] * 1.05) {
                // Saturated (e.g. current limit): keep the plateaus already measured instead of aborting.
                warn(String.format(Locale.US, "功率 %.2f 起转速不再上升，只用前 %d 档", powers[stage], stage));
                powers = java.util.Arrays.copyOf(powers, stage);
                speeds = java.util.Arrays.copyOf(speeds, stage);
                break;
            }
            report.append(String.format(Locale.US, "plateau power=%.3f rpm=%.1f sd=%.1f\n", powers[stage], speeds[stage], deviation));
        }
        double[] ff;
        try {
            ff = FlywheelTuning.fitFeedforward(speeds, powers);
        } catch (IllegalArgumentException fitFailed) {
            // Fall back to OT's form: no static term, kV = least squares through the origin.
            double sxx = 0, sxy = 0;
            for (int k = 0; k < speeds.length; k++) { sxx += speeds[k] * speeds[k]; sxy += speeds[k] * powers[k]; }
            ff = new double[] {0, sxy / sxx};
            warn("前馈拟合（" + fitFailed.getMessage() + "）不理想，改用 kS=0 的比例前馈");
        }
        if (!Double.isFinite(ff[1]) || ff[1] <= 0 || speeds[speeds.length - 1] < 100) {
            // No usable encoder data: still run the candidates with the competition KF.
            warn("测到的转速太低，前馈用比赛程序现有 KF，结果不可信，请检查 shooterMotor1 编码器");
            ff = new double[] {0, ShooterSubsystem.KF > 0 ? ShooterSubsystem.KF : .0002};
        }
        if (!Double.isFinite(ff[0]) || ff[0] < 0) ff[0] = 0;
        int top = speeds.length - 1;
        double tau = 0;
        for (double[] point : firstStep) {
            if (point[1] >= speeds[0] * .632) { tau = point[0]; break; }
        }
        if (tau < .06 || tau > 2.5) {
            warn(String.format(Locale.US, "响应时间 %.2fs 不可靠，按 0.3s 计算", tau));
            tau = .3;
        }
        double target = targetRequest > 0 ? targetRequest : speeds[top] * .8;
        if (top >= 1 && speeds[top] >= 100) {
            double clamped = Math.max(speeds[0] / .7, Math.min(speeds[top] * .9, target));
            if (clamped < speeds[top] * .9 + 1 && clamped != target) {
                warn(String.format(Locale.US, "目标 %.0f RPM 超出可调范围，改用 %.0f", target, clamped));
                target = clamped;
            }
        }
        if (!Double.isFinite(target) || target < 300) { warn("目标转速无效，按 3000 RPM"); target = 3000; }
        double baseP = tau * ff[1] / Math.max(.25, tau);
        if (!Double.isFinite(baseP) || baseP <= 0) baseP = ff[1];
        FlywheelTuning.Controller best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        int candidate = 0;
        for (double scale : new double[] {.5, 1, 2}) {
            for (double derivative : new double[] {0, .1}) {
                double p = baseP * scale;
                FlywheelTuning.Controller controller = new FlywheelTuning.Controller(p, p / tau, p * tau * derivative, ff[0], ff[1]);
                phase = "验证候选 " + (++candidate) + "/6";
                double score = trial(controller, target);
                report.append(String.format(Locale.US, "P=%.8f I=%.8f D=%.8f score=%.5f %s\n", controller.p, controller.i, controller.d, score, lastQualified ? "达标" : "未达标"));
                // Qualified candidates always beat unqualified ones; among each group, lower score wins.
                double rank = score + (lastQualified ? 0 : 1000);
                if (rank < bestScore) { bestScore = rank; best = controller; }
            }
        }
        if (best == null) best = new FlywheelTuning.Controller(baseP, baseP / tau, 0, ff[0], ff[1]);
        if (bestScore >= 1000) warn("6 组候选都没达标（误差>8% 或过冲>20% 或饱和>20%），给出误差最小的一组，需人工确认");
        phase = "复验最佳参数";
        double confirmed = trial(new FlywheelTuning.Controller(best.p, best.i, best.d, best.ks, best.kv), target);
        if (!lastQualified) warn("最佳参数复验未达标，结果仅供参考");
        result = best;
        validatedTargetRpm = target;
        report.append(String.format(Locale.US,
                "\n建议值（空载实测）\nP=%.9f\nI=%.9f\nD=%.9f\nF=kV=%.9f\nkS=%.9f\n" +
                "Target=%.1f RPM; tau=%.3fs; validated score=%.5f\n" +
                "power=kS+kV*target+P*error+I*integral-D*d(measuredRPM)/dt\n" +
                "F 与 kV 是同一个值，只加一次。dt 秒，速度 RPM。\n" +
                "20ms 周期，速度滤波 alpha=0.4，功率 [0,%.3f]，积分抗饱和。\n" +
                "比赛 ShooterSubsystem 已使用同一控制器；填写 KP/KI/KD/KF/KS，并设 MAX_POWER 为本次上限。\n",
                best.p, best.i, best.d, best.kv, best.ks, target, tau, confirmed, maxPower));
    }

    private double trial(FlywheelTuning.Controller controller, double target) throws InterruptedException {
        // Equal initial conditions for each candidate; this settling interval is not scored.
        double settling = now();
        setPower(controller.ks + controller.kv * target * .7);
        while (now() - settling < 2) sample();
        double sum = 0, overshoot = 0;
        int count = 0, saturated = 0;
        for (double fraction : new double[] {.7, 1, .85}) {
            double request = target * fraction;
            requestedRpm = request;
            double start = now();
            while (now() - start < 1.5) {
                double dt = sample();
                setPower(controller.update(request, filteredRpm, dt, maxPower));
                if (now() - start > .5) {
                    double error = (filteredRpm - request) / request;
                    sum += error * error;
                    overshoot = Math.max(overshoot, error);
                    if (controller.saturated) saturated++;
                    count++;
                }
            }
        }
        double rms = Math.sqrt(sum / Math.max(1, count));
        double saturation = (double) saturated / Math.max(1, count);
        lastQualified = count >= 20 && rms <= .08 && overshoot <= .2 && saturation <= .2;
        return rms + .25 * overshoot + .1 * saturation;
    }

    private double sample() throws InterruptedException {
        sleep(20);
        double time = now();
        if (!opModeIsActive()) throw new IllegalStateException("用户停止");
        // OT/SU do not abort on a slow loop; a Dashboard/I2C hiccup only stretches dt.
        double dt = Math.min(time - lastSample, .1);
        lastSample = time;
        // Only shooterMotor1 has an encoder. Like OT/SU getRightVelocity(), use the magnitude so the
        // encoder's wiring sign cannot abort the test (old code stopped on any reading below -100 RPM).
        double rpm = Math.abs(leader.getVelocity()) * 60 / ticksPerRev;
        if (!Double.isFinite(rpm)) { warnOnce("编码器读数无效，按 0 处理"); rpm = 0; }
        if (rpm > maxRpm) warnOnce(String.format(Locale.US,
                "shooterMotor1 读数 %.0f RPM 超过 MAX_RPM %.0f；若远超实际，TICKS_PER_REV=%.0f 可能与编码器不符",
                rpm, maxRpm, ticksPerRev));
        filteredRpm = FlywheelTuning.filter(filteredRpm, rpm, dt);
        if (commandedPower > .15 && filteredRpm < 50) {
            if (Double.isNaN(lowSpeedStarted)) lowSpeedStarted = time;
            if (time - lowSpeedStarted > 3) warnOnce(String.format(Locale.US,
                    "给了 %.2f 功率 3 秒，shooterMotor1 编码器仍 < 50 RPM（原始 %.0f ticks/s），检查编码器线",
                    commandedPower, leader.getVelocity()));
        } else {
            lowSpeedStarted = Double.NaN;
        }
        // Removed: aborting on >10% battery sag. A flywheel start alone sags the battery that much,
        // which stopped the test right after it began. OT/SU have no such check.
        if (time - lastDisplay > .2) {
            publishLive("运行中");
            lastDisplay = time;
        }
        return dt;
    }

    private void publishLive(String status) {
        telemetry.addData("状态", status);
        telemetry.addData("阶段", phase);
        telemetry.addData("使用编码器", NameConstants.SHOOTER_0);
        telemetry.addData("motor1VelocityTPS", leader == null ? 0 : leader.getVelocity());
        telemetry.addData("measuredRPM", filteredRpm);
        telemetry.addData("targetRPM", requestedRpm);
        telemetry.addData("power", commandedPower);
        telemetry.addData("elapsedSeconds", started == 0 ? 0 : now() - started);
        if (coastRemaining > 0) telemetry.addData("等待停稳剩余秒数", coastRemaining);
        telemetry.update();
    }

    private void publishResult(String outcome) {
        telemetry.addData("状态", outcome);
        telemetry.addData("停止所在阶段", phase);
        if (warnings.length() > 0) telemetry.addData("警告", "\n" + warnings);
        telemetry.addData("使用编码器", NameConstants.SHOOTER_0);
        telemetry.addData("power", 0);
        telemetry.addData("结果文件", resultFile);
        telemetry.addData("完整报告", report.toString());
        if (result != null) {
            telemetry.addData("KP", result.p);
            telemetry.addData("KI", result.i);
            telemetry.addData("KD", result.d);
            telemetry.addData("KF_equals_kV", result.kv);
            telemetry.addData("KS", result.ks);
            telemetry.addData("MAX_POWER", maxPower);
            telemetry.addData("validatedTargetRPM", validatedTargetRpm);
            telemetry.addData("TICKS_PER_REV", ticksPerRev);
        }
        telemetry.addData("下一步", "Dashboard Stop → Init → Start 可重新测试");
        telemetry.update();
    }

    /** shooterMotor1 gets -power, shooterMotor2 gets +power: same magnitude, opposite direction. */
    private void setPower(double power) {
        commandedPower = Math.max(0, Math.min(maxPower, power));
        leader.setPower(-commandedPower);
        follower.setPower(commandedPower);
    }

    private double voltage() {
        double minimum = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double value = sensor.getVoltage();
            if (value > 0) minimum = Math.min(minimum, value);
        }
        if (!Double.isFinite(minimum)) return 0;
        return minimum;
    }

    private String saveReport() {
        File directory = hardwareMap.appContext.getExternalFilesDir(null);
        if (directory == null) return "存储不可用，结果已写 Robot Controller 日志";
        File file = new File(directory, "shooter-autotune-" + System.currentTimeMillis() + ".txt");
        try (FileWriter writer = new FileWriter(file)) {
            writer.write(report.toString());
            return file.getAbsolutePath();
        } catch (IOException error) {
            return "保存失败：" + error.getMessage() + "；结果已写 Robot Controller 日志";
        }
    }

    private final java.util.Set<String> warnedOnce = new java.util.HashSet<>();

    private void warnOnce(String message) {
        String key = message.replaceAll("[0-9.]+", "#");
        if (warnedOnce.add(key)) warn(message);
    }

    private void warn(String message) {
        warnings.append("• ").append(message).append("\n");
        report.append("警告：").append(message).append("\n");
    }

    private static double now() { return System.nanoTime() * 1e-9; }
}
