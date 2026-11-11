package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NameConstants;

/** 双舵机云台：常规回中、Pinpoint 自瞄、绝对朝向锁定。 */
public class TurretSubsystem extends SubsystemBase {
    // 固定中位模式关闭，恢复 X 键 Pinpoint 云台自瞄。
    public static final boolean FIXED_POSITION_ENABLED = false;
    public static final double INIT_POSITION = 0.5;
    public static final double SOFT_LIMIT_DEG = 90.0;
    public static final double MANUAL_STEP_DEG = 2.0;

    private static final double SERVO_RANGE_DEG = 300.0;
    private static final double SOFT_HALF_POSITION = SOFT_LIMIT_DEG / SERVO_RANGE_DEG;
    private static final double DEADBAND_DEG = 0.5;
    private static final double MECHANICAL_RATIO = 0.98;
    private static final double TURRET_SIGN = 1.0;
    // 以下瞄点常量也供射击命令（shoot/ShotTarget）共用。
    public static final double MOUNT_OFFSET_DEG = 180.0;

    public static final double AIM_POINT_X_IN = 60.0;
    public static final double AIM_POINT_Y_HIGH_IN = 84.0;
    public static final double AIM_POINT_Y_LOW_IN = 60.0;
    public static final double AIM_SWITCH_Y_IN = 72.0;
    public static final double AIM_SWITCH_HYSTERESIS_IN = 1.0;
    public static final double AIM_MIN_DISTANCE_IN = 0.5;

    private final boolean fixedPositionEnabled;
    private final Servo leftServo;
    private final Servo rightServo;

    private double position = INIT_POSITION;
    private double targetYawDeg = LocalizationSubsystem.START_HEADING_DEG;
    private double aimOffsetDeg;
    private boolean aimMode;
    private boolean lockMode;

    private double aimDistance;
    private double aimTargetDeg;
    private double aimTargetX = AIM_POINT_X_IN;
    private double aimTargetY = AIM_POINT_Y_LOW_IN;
    private boolean hasValidAim;
    private double lastValidAimTargetDeg;
    private boolean reachable = true;
    private double yawErrorDeg;
    private double effectiveTargetDeg;
    private double actualAbsoluteDeg;
    private double rawPosition = INIT_POSITION;

    public TurretSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, FIXED_POSITION_ENABLED);
    }

    /** 模式在构造时确定，避免运行中切换造成舵机跳转；布尔参数也供离线验证使用。 */
    public TurretSubsystem(HardwareMap hardwareMap, boolean fixedPositionEnabled) {
        this.fixedPositionEnabled = fixedPositionEnabled;
        leftServo = hardwareMap.get(Servo.class, NameConstants.TURRET_LEFT);
        rightServo = hardwareMap.get(Servo.class, NameConstants.TURRET_RIGHT);
        // 左右舵机按镜像安装配置；实机确认转向和中位。
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.FORWARD);
        setServoPositions(INIT_POSITION);
    }

    public boolean toggleAimMode(boolean pinpointReady) {
        return setAimMode(!aimMode, pinpointReady);
    }

    public boolean setAimMode(boolean enabled, boolean pinpointReady) {
        if (fixedPositionEnabled) {
            holdInitialPosition();
            return !enabled;
        }
        if (enabled && !pinpointReady) {
            return false;
        }
        if (aimMode == enabled) {
            return true;
        }
        aimMode = enabled;
        if (aimMode) {
            lockMode = false;
        }
        hasValidAim = false;
        lastValidAimTargetDeg = 0.0;
        return true;
    }

    public void toggleLockMode(double robotYawDeg) {
        if (fixedPositionEnabled) {
            holdInitialPosition();
            return;
        }
        lockMode = !lockMode;
        if (lockMode) {
            aimMode = false;
            hasValidAim = false;
            targetYawDeg = normalizeDegrees(robotYawDeg + MOUNT_OFFSET_DEG
                    + TURRET_SIGN * (position - INIT_POSITION)
                    * SERVO_RANGE_DEG / MECHANICAL_RATIO);
        }
    }

    public void adjustLeft() {
        if (fixedPositionEnabled) return;
        if (aimMode) {
            aimOffsetDeg = normalizeDegrees(aimOffsetDeg + MANUAL_STEP_DEG);
        } else if (lockMode) {
            targetYawDeg = normalizeDegrees(targetYawDeg + MANUAL_STEP_DEG);
        }
    }

    public void adjustRight() {
        if (fixedPositionEnabled) return;
        if (aimMode) {
            aimOffsetDeg = normalizeDegrees(aimOffsetDeg - MANUAL_STEP_DEG);
        } else if (lockMode) {
            targetYawDeg = normalizeDegrees(targetYawDeg - MANUAL_STEP_DEG);
        }
    }

    public void reset() {
        if (fixedPositionEnabled) {
            holdInitialPosition();
            return;
        }
        targetYawDeg = LocalizationSubsystem.START_HEADING_DEG;
        aimOffsetDeg = 0.0;
        hasValidAim = false;
        lastValidAimTargetDeg = 0.0;
        aimMode = false;
        lockMode = false;
    }

    public void update(LocalizationSubsystem localization) {
        double yawRad = localization.getYawRadians();
        double yawDeg = Math.toDegrees(yawRad);
        if (fixedPositionEnabled) {
            holdInitialPosition();
            actualAbsoluteDeg = normalizeDegrees(yawDeg + MOUNT_OFFSET_DEG);
            updateAimDistance(localization);
            return;
        }

        if (localization.isPinpointReady()) {
            updateAimPoint(localization.getYIn());
        }

        yawErrorDeg = 0.0;
        effectiveTargetDeg = 0.0;
        actualAbsoluteDeg = 0.0;
        rawPosition = INIT_POSITION;
        reachable = true;

        if (aimMode && localization.isPinpointReady()) {
            updateAutoAim(localization, yawRad, yawDeg);
        } else if (lockMode) {
            updateLock(yawRad, yawDeg);
            updateAimDistance(localization);
        } else {
            position = INIT_POSITION;
            rawPosition = INIT_POSITION;
            actualAbsoluteDeg = normalizeDegrees(yawDeg + MOUNT_OFFSET_DEG);
            updateAimDistance(localization);
        }

        setServoPositions(position);
    }

    /** 固定舵机位置，不是固定场地绝对朝向；转动车身时云台不补偿。 */
    private void holdInitialPosition() {
        position = rawPosition = INIT_POSITION;
        aimMode = lockMode = hasValidAim = false;
        aimOffsetDeg = lastValidAimTargetDeg = yawErrorDeg = effectiveTargetDeg = 0.0;
        targetYawDeg = LocalizationSubsystem.START_HEADING_DEG;
        reachable = true;
        setServoPositions(INIT_POSITION);
    }

    public boolean isFixedPositionEnabled() {
        return fixedPositionEnabled;
    }

    private void updateAimPoint(double robotYIn) {
        if (aimTargetY == AIM_POINT_Y_LOW_IN) {
            if (robotYIn > AIM_SWITCH_Y_IN + AIM_SWITCH_HYSTERESIS_IN) {
                aimTargetY = AIM_POINT_Y_HIGH_IN;
            }
        } else if (robotYIn < AIM_SWITCH_Y_IN - AIM_SWITCH_HYSTERESIS_IN) {
            aimTargetY = AIM_POINT_Y_LOW_IN;
        }
    }

    private void updateAutoAim(LocalizationSubsystem localization, double yawRad, double yawDeg) {
        double dx = aimTargetX - localization.getXIn();
        double dy = aimTargetY - localization.getYIn();
        aimDistance = Math.hypot(dx, dy);

        if (aimDistance > AIM_MIN_DISTANCE_IN) {
            aimTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
            lastValidAimTargetDeg = aimTargetDeg;
            hasValidAim = true;
            targetYawDeg = normalizeDegrees(aimTargetDeg + aimOffsetDeg);
        } else if (hasValidAim) {
            aimTargetDeg = lastValidAimTargetDeg;
            targetYawDeg = normalizeDegrees(aimTargetDeg + aimOffsetDeg);
        } else {
            double currentAbsoluteDeg = getCurrentAbsoluteDegrees(yawDeg);
            targetYawDeg = currentAbsoluteDeg;
            aimTargetDeg = normalizeDegrees(currentAbsoluteDeg - aimOffsetDeg);
        }

        updateServoTarget(yawRad, yawDeg);
    }

    /** Field heading that points the centered launcher at the existing aim point. */
    public double getChassisAimHeadingRadians(LocalizationSubsystem localization) {
        double x = localization.getXIn();
        double y = localization.getYIn();
        if (!Double.isFinite(x) || !Double.isFinite(y)) return Double.NaN;
        updateAimPoint(y);
        double dx = aimTargetX - x;
        double dy = aimTargetY - y;
        if (Math.hypot(dx, dy) <= AIM_MIN_DISTANCE_IN) return Double.NaN;
        return Math.atan2(dy, dx) - Math.toRadians(MOUNT_OFFSET_DEG);
    }

    private void updateLock(double yawRad, double yawDeg) {
        updateServoTarget(yawRad, yawDeg);
    }

    private void updateServoTarget(double yawRad, double yawDeg) {
        effectiveTargetDeg = normalizeDegrees(targetYawDeg + MOUNT_OFFSET_DEG);
        double targetRad = Math.toRadians(effectiveTargetDeg);
        yawErrorDeg = Math.toDegrees(normalizeRadians(targetRad - yawRad));
        if (Math.abs(yawErrorDeg) < DEADBAND_DEG) {
            yawErrorDeg = 0.0;
        }

        double relativeDeg = TURRET_SIGN * yawErrorDeg / MECHANICAL_RATIO;
        rawPosition = INIT_POSITION + relativeDeg / SERVO_RANGE_DEG;
        reachable = Math.abs(rawPosition - INIT_POSITION) <= SOFT_HALF_POSITION;
        position = clamp(rawPosition,
                INIT_POSITION - SOFT_HALF_POSITION,
                INIT_POSITION + SOFT_HALF_POSITION);
        actualAbsoluteDeg = getCurrentAbsoluteDegrees(yawDeg);
    }

    private double getCurrentAbsoluteDegrees(double robotYawDeg) {
        return normalizeDegrees(robotYawDeg + MOUNT_OFFSET_DEG
                + TURRET_SIGN * (position - INIT_POSITION)
                * SERVO_RANGE_DEG / MECHANICAL_RATIO);
    }

    private void updateAimDistance(LocalizationSubsystem localization) {
        if (localization.isPinpointReady()) {
            aimDistance = Math.hypot(
                    aimTargetX - localization.getXIn(),
                    aimTargetY - localization.getYIn());
        }
    }

    private void setServoPositions(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    public static double normalizeDegrees(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    public boolean isAimMode() {
        return aimMode;
    }

    public boolean isLockMode() {
        return lockMode;
    }

    public double getPosition() {
        return position;
    }

    public double getTargetYawDeg() {
        return targetYawDeg;
    }

    public double getActualAbsoluteDeg() {
        return actualAbsoluteDeg;
    }

    public double getYawErrorDeg() {
        return yawErrorDeg;
    }

    public boolean isReachable() {
        return reachable;
    }

    public double getAimDistance() {
        return aimDistance;
    }

    public double getAimTargetDeg() {
        return aimTargetDeg;
    }

    public double getAimTargetX() {
        return aimTargetX;
    }

    public double getAimTargetY() {
        return aimTargetY;
    }

    public double getAimOffsetDeg() {
        return aimOffsetDeg;
    }

    public boolean hasValidAim() {
        return hasValidAim;
    }

    public double getMinServoPosition() {
        return INIT_POSITION - SOFT_HALF_POSITION;
    }

    public double getMaxServoPosition() {
        return INIT_POSITION + SOFT_HALF_POSITION;
    }
}
