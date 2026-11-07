package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.NameConstants;

<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subSystems/servo.java
/** 基于 Pinpoint 场地坐标的双 Hive 云台自瞄。 */
public class servo {
=======
public class TurretSubsystem {
>>>>>>> 615cbc26a9b7eb324187e0c7060f7ea28532f84f:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/TurretSubsystem.java

    /* ==================== 云台参数 ==================== */
    private static final double TURRET_INIT = 0.5; // 舵机初始中位位置
    private static final double TURRET_MIN = 0.0; // 舵机最小位置
    private static final double TURRET_MAX = 1.0; // 舵机最大位置
    private static final double TURRET_SERVO_RANGE_DEG = 300.0; // 舵机总行程，单位为度
    private static final double TURRET_MANUAL_STEP_DEG = 2.0; // 每次手动微调的角度
    private static final double TURRET_DEADBAND_DEG = 0.5; // 云台角度误差死区
    private static final double TURRET_MECH_RATIO = 0.98; // 同步带传动比 147:150
    private static final double TURRET_SIGN = 1.0; // 云台旋转方向，反向时改为 -1.0
    private static final double TURRET_MOUNT_OFFSET_DEG = 180.0; // 云台机械安装偏移角

    /* ==================== 初始场地位置 ==================== */
    private static final double START_X_IN = 72.0; // 初始 X 坐标，单位为英寸
    private static final double START_Y_IN = 72.0; // 初始 Y 坐标，单位为英寸
    private static final double START_HEADING_DEG = 0.0; // 初始场地朝向，单位为度

    /* ==================== 双 Hive 瞄点 ==================== */
    private static final double AIM_POINT_X_IN = 60.0; // 两个 Hive 瞄点的 X 坐标
    private static final double AIM_POINT_Y_HIGH_IN = 84.0; // 高侧 Hive 瞄点的 Y 坐标
    private static final double AIM_POINT_Y_LOW_IN = 60.0; // 低侧 Hive 瞄点的 Y 坐标
    private static final double AIM_SWITCH_Y_IN = 72.0; // 高低瞄点的切换基准 Y 坐标
    private static final double AIM_SWITCH_HYST_IN = 1.0; // 瞄点切换滞回，防止抖动
    private static final double AIM_MIN_DIST_IN = 0.5; // 计算有效瞄准方向的最小距离

    private final Servo turretLeft;
    private final Servo turretRight;
    private final GoBildaPinpointDriver pinpoint;

    private double turretPos = TURRET_INIT;
    private double turretTargetYawDeg = START_HEADING_DEG;
    private double aimOffsetDeg = 0.0;

    private boolean prevLeft;
    private boolean prevRight;
    private boolean prevStart;
    private boolean prevX;

<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subSystems/servo.java
    private boolean needSetStartPos = true;
    private double pinX = START_X_IN;
    private double pinY = START_Y_IN;
    private double pinHeadingDeg = START_HEADING_DEG;

    private boolean aimMode = false;
    private double aimDistance = 0.0;
    private double aimTargetDeg = 0.0;
    private double aimTargetX = AIM_POINT_X_IN;
    private double aimTargetY = AIM_POINT_Y_LOW_IN;

    private boolean hasValidAim = false;
    private double lastValidAimTargetDeg = 0.0;
    private boolean turretReachable = true;

    public servo(HardwareMap hardwareMap) {
        turretLeft = hardwareMap.get(Servo.class, NameConstants.SERVO_LEFT);
        turretRight = hardwareMap.get(Servo.class, NameConstants.SERVO_RIGHT);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, NameConstants.OD);
        setBothPosition(TURRET_INIT);
=======
    // 使用 FTC SDK 舵机类型
    private com.qualcomm.robotcore.hardware.Servo servoL;
    private com.qualcomm.robotcore.hardware.Servo servoR;

    /**
     * 初始化两个舵机和 Pinpoint。
     * 注意：这里只读取 od 轮反馈的角度，不会调用 resetPosAndIMU，
     * 以免清零底盘正在使用的定位数据。
     */
    public TurretSubsystem(HardwareMap hardwareMap) {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, OD_NAME);

        servoL = hardwareMap.get(
                com.qualcomm.robotcore.hardware.Servo.class,
                SERVO_LEFT_NAME
        );

        servoR = hardwareMap.get(
                com.qualcomm.robotcore.hardware.Servo.class,
                SERVO_RIGHT_NAME
        );

        // 初始化时，让两个舵机来到150°中位
        setBothPosition(
                SERVO_CENTER_DEG / SERVO_TRAVEL_DEG
        );
>>>>>>> 615cbc26a9b7eb324187e0c7060f7ea28532f84f:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/TurretSubsystem.java
    }

    /** 在 waitForStart() 后调用，复位按键边沿和云台状态。 */
    public void start() {
        turretPos = TURRET_INIT;
        turretTargetYawDeg = START_HEADING_DEG;
        aimOffsetDeg = 0.0;
        aimMode = false;
        aimDistance = 0.0;
        aimTargetDeg = 0.0;
        aimTargetX = AIM_POINT_X_IN;
        aimTargetY = AIM_POINT_Y_LOW_IN;
        hasValidAim = false;
        lastValidAimTargetDeg = 0.0;
        turretReachable = true;
        needSetStartPos = true;

        prevLeft = false;
        prevRight = false;
        prevX = false;
        // 避免用于启动 OpMode 的 START 被误认为一次重置命令。
        prevStart = true;
        setBothPosition(TURRET_INIT);
    }

    /**
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subSystems/servo.java
     * 每圈调用一次。
     * X：切换双 Hive 自瞄；dpad 左右：微调 2°；START：重置定位和云台。
     * 自瞄目标超出舵机行程时，手柄持续震动提醒。
     */
    public void update(Gamepad gamepad, Telemetry telemetry) {
=======
     * 在RobotTeleOp主循环中反复调用。
     *
     * @param toZero 三角键Y：按住时两个舵机转到0
     * @param toOne  圆圈键B：按住时两个舵机转到1
     * @param reset   回中：两个舵机转到0.5（150°中位）
     *
     * 没有按下按键时，执行 od 角度自动反向补偿。
     * 如果两个按键同时按下，toZero优先。
     */
    public void update(boolean toZero, boolean toOne, boolean reset) {

        // 手动控制优先于自动控制
        if (toZero) {
            setBothPosition(0.0);
            return;
        }

        if (toOne) {
            setBothPosition(1.0);
            return;
        }

        if (reset) {
            setBothPosition(0.5);
            return;
        }

        // 读取 od 轮反馈的、相对初始方向的转角
>>>>>>> 615cbc26a9b7eb324187e0c7060f7ea28532f84f:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/TurretSubsystem.java
        pinpoint.update();

        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        if (needSetStartPos && status == GoBildaPinpointDriver.DeviceStatus.READY) {
            resetPinpointPosition();
            needSetStartPos = false;
        }

        Pose2D pose = pinpoint.getPosition();
        pinX = pose.getX(DistanceUnit.INCH);
        pinY = pose.getY(DistanceUnit.INCH);
        pinHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

        updateAimPoint();

        // X 上升沿切换自瞄，并清除上一瞄准方向。
        boolean x = gamepad.x;
        if (x && !prevX) {
            aimMode = !aimMode;
            hasValidAim = false;
            lastValidAimTargetDeg = 0.0;
        }
        prevX = x;

        updateAutoAim();
        updateManualTrim(gamepad);

        // 使用 od 朝向计算云台需要反向补偿的角度。
        double effectiveTargetDeg = normalizeDeg(
                turretTargetYawDeg + TURRET_MOUNT_OFFSET_DEG);
        double yawErrorDeg = normalizeDeg(effectiveTargetDeg - pinHeadingDeg);
        if (Math.abs(yawErrorDeg) < TURRET_DEADBAND_DEG) {
            yawErrorDeg = 0.0;
        }

        double turretRelativeDeg = TURRET_SIGN * yawErrorDeg / TURRET_MECH_RATIO;
        double turretPosRaw = TURRET_INIT
                + turretRelativeDeg / TURRET_SERVO_RANGE_DEG;
        turretReachable = turretPosRaw >= TURRET_MIN && turretPosRaw <= TURRET_MAX;
        turretPos = Range.clip(turretPosRaw, TURRET_MIN, TURRET_MAX);
        setBothPosition(turretPos);

        double turretActualAbsDeg = normalizeDeg(
                pinHeadingDeg
                        + TURRET_MOUNT_OFFSET_DEG
                        + TURRET_SIGN * (turretPos - TURRET_INIT)
                        * TURRET_SERVO_RANGE_DEG / TURRET_MECH_RATIO);

        if (aimMode && !turretReachable) {
            gamepad.rumble(0.8, 0.8, 200);
        } else {
            gamepad.stopRumble();
        }

        // START 上升沿恢复中心起点和云台默认状态。
        boolean startButton = gamepad.start;
        if (startButton && !prevStart) {
            resetPinpointPosition();
            needSetStartPos = false;
            pinX = START_X_IN;
            pinY = START_Y_IN;
            pinHeadingDeg = START_HEADING_DEG;
            turretTargetYawDeg = START_HEADING_DEG;
            aimOffsetDeg = 0.0;
            hasValidAim = false;
            lastValidAimTargetDeg = 0.0;
        }
        prevStart = startButton;

        addTelemetry(telemetry, status, effectiveTargetDeg,
                turretActualAbsDeg, yawErrorDeg, turretPosRaw);
    }

    /** 在 y=72 附近使用 1 英寸滞回，防止瞄点来回跳变。 */
    private void updateAimPoint() {
        if (aimTargetY == AIM_POINT_Y_LOW_IN) {
            if (pinY > AIM_SWITCH_Y_IN + AIM_SWITCH_HYST_IN) {
                aimTargetY = AIM_POINT_Y_HIGH_IN;
            }
        } else if (pinY < AIM_SWITCH_Y_IN - AIM_SWITCH_HYST_IN) {
            aimTargetY = AIM_POINT_Y_LOW_IN;
        }
    }

    private void updateAutoAim() {
        double dx = aimTargetX - pinX;
        double dy = aimTargetY - pinY;
        aimDistance = Math.hypot(dx, dy);

        if (!aimMode) {
            return;
        }

        if (aimDistance > AIM_MIN_DIST_IN) {
            aimTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
            lastValidAimTargetDeg = aimTargetDeg;
            hasValidAim = true;
            turretTargetYawDeg = normalizeDeg(aimTargetDeg + aimOffsetDeg);
        } else if (hasValidAim) {
            // 太靠近瞄点时沿用最后一次有效方向。
            aimTargetDeg = lastValidAimTargetDeg;
            turretTargetYawDeg = normalizeDeg(aimTargetDeg + aimOffsetDeg);
        } else {
            // 尚未建立有效方向时保持当前云台朝向。
            double currentTurretAbsDeg = normalizeDeg(
                    pinHeadingDeg
                            + TURRET_MOUNT_OFFSET_DEG
                            + TURRET_SIGN * (turretPos - TURRET_INIT)
                            * TURRET_SERVO_RANGE_DEG / TURRET_MECH_RATIO);
            turretTargetYawDeg = currentTurretAbsDeg;
            aimTargetDeg = normalizeDeg(currentTurretAbsDeg - aimOffsetDeg);
        }
    }

    private void updateManualTrim(Gamepad gamepad) {
        boolean left = gamepad.dpad_left;
        boolean right = gamepad.dpad_right;

        if (left && !prevLeft) {
            if (aimMode) {
                aimOffsetDeg = normalizeDeg(aimOffsetDeg + TURRET_MANUAL_STEP_DEG);
            } else {
                turretTargetYawDeg = normalizeDeg(
                        turretTargetYawDeg + TURRET_MANUAL_STEP_DEG);
            }
        }
        if (right && !prevRight) {
            if (aimMode) {
                aimOffsetDeg = normalizeDeg(aimOffsetDeg - TURRET_MANUAL_STEP_DEG);
            } else {
                turretTargetYawDeg = normalizeDeg(
                        turretTargetYawDeg - TURRET_MANUAL_STEP_DEG);
            }
        }
        prevLeft = left;
        prevRight = right;
    }

    private void resetPinpointPosition() {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                START_X_IN,
                START_Y_IN,
                AngleUnit.DEGREES,
                START_HEADING_DEG));
    }

    private void setBothPosition(double position) {
        double safePosition = Range.clip(position, TURRET_MIN, TURRET_MAX);
        turretLeft.setPosition(safePosition);
        turretRight.setPosition(safePosition);
    }

    private void addTelemetry(Telemetry telemetry,
                              GoBildaPinpointDriver.DeviceStatus status,
                              double effectiveTargetDeg,
                              double turretActualAbsDeg,
                              double yawErrorDeg,
                              double turretPosRaw) {
        telemetry.addData("Pinpoint 状态", status);
        telemetry.addData("Pinpoint X/Y (in)", "(%.2f, %.2f)", pinX, pinY);
        telemetry.addData("Pinpoint Heading (deg)", "%.1f", pinHeadingDeg);
        telemetry.addData("设置起点", needSetStartPos ? "等待 READY" : "完成");
        telemetry.addData("自瞄模式", aimMode ? "开 (X 关闭)" : "关 (X 开启)");
        telemetry.addData("当前 Hive 瞄点 (in)", "(%.1f, %.1f)", aimTargetX, aimTargetY);
        telemetry.addData("自瞄距离 (in)", "%.2f", aimDistance);
        telemetry.addData("指向瞄点绝对角 (deg)", "%.1f", aimTargetDeg);
        telemetry.addData("自瞄偏移角 (deg)", "%.1f", aimOffsetDeg);
        telemetry.addData("有效瞄准已建立", hasValidAim ? "是" : "否");
        telemetry.addData("云台目标绝对角 (deg)", "%.1f", turretTargetYawDeg);
        telemetry.addData("送给舵机目标角 (deg)", "%.1f", effectiveTargetDeg);
        telemetry.addData("云台实际绝对角 (deg)", "%.1f", turretActualAbsDeg);
        telemetry.addData("云台锁定误差 (deg)", "%.1f", yawErrorDeg);
        telemetry.addData("云台舵机位置", "%.3f", turretPos);
        telemetry.addData("云台原始位置(未限幅)", "%.3f", turretPosRaw);
        telemetry.addData("云台可达", turretReachable ? "是" : "否（已限幅并震动）");

        if (aimMode && !hasValidAim) {
            telemetry.addLine("距离瞄点太近，方向未定义，请移开机器人");
        }
    }

    private static double normalizeDeg(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
