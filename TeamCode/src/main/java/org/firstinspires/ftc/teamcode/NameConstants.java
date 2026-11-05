package org.firstinspires.ftc.teamcode;

/**
 * 所有电机在 Driver Station（Configure Robot）里的配置名称。
 * 改名只需要动这里，其余文件全部引用本类。
 */
public final class NameConstants {

    private NameConstants() {}

    /* ==================== 底盘（MecanumDrive / AutoPath 共用） ==================== */
    public static final String LEFT_FRONT_MOTOR  = "leftFrontMotor";
    public static final String RIGHT_FRONT_MOTOR = "rightFrontMotor";
    public static final String LEFT_BACK_MOTOR   = "leftBackMotor";
    public static final String RIGHT_BACK_MOTOR  = "rightBackMotor";

    /* ==================== 发射机构（subSystems.shooter） ==================== */
    public static final String SHOOTER_MOTOR_0 = "0";   // 飞轮 0（带编码器读转速）
    public static final String SHOOTER_MOTOR_1 = "1";   // 飞轮 1
    public static final String SHOOTER_MOTOR_2 = "2";   // 供弹 2
    public static final String SHOOTER_MOTOR_3 = "3";   // 供弹 3

    /* ==================== 入料（subSystems.inTake） ==================== */
    public static final String INTAKE_MOTOR = "intakeMotor";
    public static final String TRANSIT_MOTOR = "transitMotor";

    /* ==================== 定位与视觉 ==================== */
    public static final String OD        = "od";        // goBILDA Pinpoint 定位
    public static final String LIMELIGHT = "limelight"; // Limelight 相机

    /* ==================== 舵机与 IMU（subSystems.servo） ==================== */
    public static final String SERVO_LEFT  = "leftServo";
    public static final String SERVO_RIGHT = "rightServo";
    public static final String IMU         = "imu";     // Control Hub 内置 IMU
}
