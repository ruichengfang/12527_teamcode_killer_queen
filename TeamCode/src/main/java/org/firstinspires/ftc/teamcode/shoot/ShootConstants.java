package org.firstinspires.ftc.teamcode.shoot;

import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;

/**
 * 移植自 Overtune 射击命令的参数表。逻辑与 OT 相同，数值优先取 KQ 现有值；
 * 标注“OT 默认”的项 KQ 原先没有对应参数，沿用 OT 数值，待实车标定。
 */
public final class ShootConstants {
    // ---- 手柄 ----
    /** KQ DriveCommand 摇杆死区。 */
    public static final double JOYSTICK_DEADBAND = 0.05;
    /** KQ IntakeCommand/IntakeSubsystem 扳机阈值（>0.05）。 */
    public static final double TRIGGER_THRESHOLD = 0.05;

    // ---- 瞄准几何 ----
    /**
     * 发射口相对车头的偏角。用户确认 KQ 飞轮出口与 OT 相同朝车尾，故直接沿用 OT
     * VisionConstants.shooterYawDegrees = 180°；不引用 KQ 云台/旧底盘自瞄的约定。
     */
    public static final double SHOOTER_YAW_DEGREES = 180.0;

    // ---- 底盘自瞄 AutoAimController（OT 符号约定：Kp<0 时正误差输出逆时针转向）----
    /** KQ AimAndShootCommand.TURN_POWER_PER_RADIAN = 1.0。 */
    public static final double AUTO_AIM_KP = -1.0;
    public static final double AUTO_AIM_KI = 0.0;
    /** KQ 原自瞄为纯 P 控制，无 D 项。 */
    public static final double AUTO_AIM_KD = 0.0;
    public static final double AUTO_AIM_KF = 0.0;
    /** KQ MIN_TURN_POWER = 0.07，作为容差外的静摩擦补偿。 */
    public static final double AUTO_AIM_KS = 0.07;
    /** KQ 底盘正 turn 为顺时针，与 OT moveRobot 相同，故符号保持 1。 */
    public static final double AUTO_AIM_YAW_TO_TURN_SIGN = 1.0;
    /** KQ MAX_TURN_POWER = 0.40。 */
    public static final double AUTO_AIM_MAX_TURN_POWER = 0.40;
    public static final double AUTO_AIM_TEST_MAX_TURN_POWER = 0.40;
    /** OT 默认（仅未校准搜索分支使用，手动射击不会调用），仍受 0.40 上限限制。 */
    public static final double AUTO_AIM_ACQUISITION_TURN_POWER = 0.65;
    /** KQ AIM_TOLERANCE = 3°。 */
    public static final double AIM_LOCK_TOLERANCE_DEG = 3.0;
    /** OT 默认迟滞宽度 1°（OT 为 2°/3°），叠加在 KQ 的 3° 上。 */
    public static final double AIM_UNLOCK_TOLERANCE_DEG = 4.0;
    /** OT 默认。 */
    public static final double AIM_LOCK_STABLE_SECONDS = 0.1;

    // ---- 站桩位置保持 PositionHoldController（KQ 无对应功能，全部 OT 默认）----
    public static final boolean POSITION_HOLD_CALIBRATED = true;
    public static final double POSITION_HOLD_KP = 0.12;
    public static final double POSITION_HOLD_KI = 0.0;
    public static final double POSITION_HOLD_KD = 0.007;
    public static final double POSITION_HOLD_MAX_POWER = 0.6;
    public static final double POSITION_LOCK_ENTER_TOLERANCE_IN = 0.75;
    public static final double POSITION_LOCK_EXIT_TOLERANCE_IN = 0.75;
    public static final double POSITION_LOCK_MAX_SPEED_IN_PER_SEC = 2.0;
    public static final double POSITION_LOCK_STABLE_SECONDS = 0.15;

    // ---- Transit 送料状态机（门 = KQ blockerServo）----
    public static final boolean LIMIT_POSITIONS_CALIBRATED = true;
    public static final double LIMIT_CLOSED = BlockerSubsystem.CLOSED_POSITION;
    public static final double LIMIT_OPEN = BlockerSubsystem.OPEN_POSITION;
    /** KQ 原联合发射送料：transit 1.0。 */
    public static final double TRANSIT_FEED_POWER = 1.0;
    /** KQ LT 满扳机反转：transit -1.0。 */
    public static final double TRANSIT_REVERSE_POWER = -1.0;
    /** OT 默认（LB 未绑定，当前不会调用）。 */
    public static final double TRANSIT_INTAKE_REVERSE_FORWARD_POWER = 0.6;
    /** KQ RT 满扳机正转：transit 1.0。 */
    public static final double TRANSIT_FORWARD_POWER = 1.0;
    /** OT 默认：开门后先短暂反转。 */
    public static final double FEED_REVERSE_POWER = -0.4;
    public static final double FEED_REVERSE_TIME_SECONDS = 0.1;

    // ---- Intake 射击辅助 ----
    /** KQ RB 单独吸入：intake 1.0。 */
    public static final double INTAKE_ACTIVE_POWER = 1.0;
    /** KQ 原联合发射送料：intake 1.0。 */
    public static final double INTAKE_FEED_POWER = 1.0;
    /** KQ LT 满扳机反转：intake -1.0。 */
    public static final double INTAKE_JAM_POWER = -1.0;

    private ShootConstants() {
    }
}
