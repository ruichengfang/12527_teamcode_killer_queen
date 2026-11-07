//package org.firstinspires.ftc.teamcode.subsystem;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.NameConstants;
//
///**
// * Shooter 子系统
// *
// * 【需要自行修改的部分】
// * 1. MOTOR_0~3：改成你在 Driver Station 里配置的设备名
// * 2. motor0/motor1 方向：根据实际安装调整 REVERSE / FORWARD
// * 3. motor3 方向：若物理上与电机 2 反向安装，取消注释 setDirection(REVERSE)
// * 4. TICKS_PER_REV：按你的电机编码器规格修改（裸电机 28，带齿轮箱需乘减速比）
// * 5. PID 参数 kP / kD / kF：需根据实际飞轮调参
// * 6. BUMPER_FEED_POWER / TRIGGER_MAX_POWER：按需调整
// */
//public class Shooter {
//
//    /* ==================== 设备名（和 Configure Robot 里完全一致） ==================== */
//    private static final String MOTOR_0 = NameConstants.SHOOTER_MOTOR_0;
//    private static final String MOTOR_1 = NameConstants.SHOOTER_MOTOR_1;
//    private static final String MOTOR_2 = NameConstants.SHOOTER_MOTOR_2;
//    private static final String MOTOR_3 = NameConstants.SHOOTER_MOTOR_3;
//
//    /* ==================== 编码器 / 转速 ==================== */
//    private static final double TICKS_PER_REV = 28.0;    // 【需修改】按电机实际规格填写
//
//    private static final double MIN_RPM  = 0;
//    private static final double MAX_RPM  = 6000;
//    private static final double RPM_RATE = 1000.0;       // ↑/↓ 按住时每秒变化多少 RPM
//
//    /** 扳机压满（1.0）时电机 2、3 的功率。 */
//    private static final double TRIGGER_MAX_POWER = 1.0;
//
//    /** 按住右 Bumper 时 2、3 电机的固定正转功率。 */
//    private static final double BUMPER_FEED_POWER = 1;
//
//    /* ==================== PID 参数（仅 0/1 使用） ==================== */
//    private double kP = 0.0020;   // 【需修改】按实际飞轮调参
//    private double kD = 0.00002;  // 【需修改】按实际飞轮调参
//    private double kF = 0.00019;  // 【需修改】按实际飞轮调参
//
//    /* ==================== 硬件 ==================== */
//    private DcMotorEx motor0, motor1, motor2, motor3;
//
//    /* ==================== 状态 ==================== */
//    private double targetRPM01   = 0;
//    private double filteredRPM01 = 0;
//    private double lastError01   = 0;
//    private double power01       = 0;
//    private double power2 = 0;
//    private double power3 = 0;
//    private double lastTime = 0;
//
//    /* ==================== 初始化 ==================== */
//    public Shooter(HardwareMap hardwareMap) {
//        motor0 = hardwareMap.get(DcMotorEx.class, MOTOR_0);
//        motor1 = hardwareMap.get(DcMotorEx.class, MOTOR_1);
//        motor2 = hardwareMap.get(DcMotorEx.class, MOTOR_2);
//        motor3 = hardwareMap.get(DcMotorEx.class, MOTOR_3);
//
//        for (DcMotorEx m : new DcMotorEx[]{motor0, motor1, motor2, motor3}) {
//            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        }
//
//        // 【需修改】飞轮方向修正
//        motor0.setDirection(DcMotor.Direction.REVERSE);
//        motor1.setDirection(DcMotor.Direction.FORWARD);
//
//        // 【需修改】电机 3 若物理上与电机 2 反向安装，取消注释：
//        // motor3.setDirection(DcMotor.Direction.REVERSE);
//    }
//
//    /**
//     * 每周期调用一次，更新所有电机的 setPower。
//     *
//     * @param runtime        传入 getRuntime()，用于计算 dt
//     * @param dpadUp         十字键上：提高 0/1 目标转速
//     * @param dpadDown       十字键下：降低 0/1 目标转速
//     * @param a              A 键：清零 0/1 目标转速
//     * @param rightBumper    右 Bumper：强制 2、3 正转
//     * @param b              B 键：反转修饰（配合扳机）
//     * @param rightTrigger   右扳机：控制电机 2
//     * @param leftTrigger    左扳机：控制电机 3
//     */
//    public void update(double runtime,
//                       boolean dpadUp, boolean dpadDown, boolean a,
//                       boolean rightBumper, boolean b,
//                       double rightTrigger, double leftTrigger) {
//
//        double dt = runtime - lastTime;
//        if (dt < 1e-4) dt = 1e-4;
//        lastTime = runtime;
//
//        // ---------- 1. 电机 0、1：十字键调目标转速 + A 清零 ----------
//        if (dpadUp)   targetRPM01 += RPM_RATE * dt;
//        if (dpadDown) targetRPM01 -= RPM_RATE * dt;
//        if (a)        targetRPM01 = 0;
//        targetRPM01 = clamp(targetRPM01, MIN_RPM, MAX_RPM);
//
//        // ---------- 2. 电机 2、3 功率计算 ----------
//        if (rightBumper) {
//            power2 = BUMPER_FEED_POWER;
//            power3 = BUMPER_FEED_POWER;
//        } else {
//            double dir = b ? -1.0 : 1.0;
//            power2 = dir * rightTrigger * TRIGGER_MAX_POWER;
//            power3 = dir * leftTrigger * TRIGGER_MAX_POWER;
//        }
//
//        // ---------- 3. 0/1 组：读 0 号转速 → PD → 两电机同功率 ----------
//        double raw01 = motor0.getVelocity() * 60.0 / TICKS_PER_REV;
//        filteredRPM01 += 0.4 * (raw01 - filteredRPM01);
//
//        double err01  = targetRPM01 - filteredRPM01;
//        double derr01 = (err01 - lastError01) / dt;
//        lastError01   = err01;
//
//        power01 = clamp(kF * targetRPM01 + kP * err01 + kD * derr01, -1.0, 1.0);
//
//        // ---------- 4. 所有电机 setPower ----------
//        motor0.setPower(power01);
//        motor1.setPower(power01);
//        motor2.setPower(power2);
//        motor3.setPower(power3);
//    }
//
//    /* ==================== 遥测读取（可选，主程序 telemetry 用） ==================== */
//    public double getTargetRPM()   { return targetRPM01; }
//    public double getFilteredRPM() { return filteredRPM01; }
//    public double getPower01()     { return power01; }
//    public double getPower2()      { return power2; }
//    public double getPower3()      { return power3; }
//
//    /* ==================== 工具方法 ==================== */
//    private static double clamp(double v, double lo, double hi) {
//        return Math.max(lo, Math.min(hi, v));
//    }
//}
