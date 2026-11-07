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

/** 云台三态控制：常规固定 0.5、X 双 Hive 自瞄、Y 场地方向锁定。 */
public class TurretSubsystem {

    private static final double TURRET_INIT = 0.5; // 常规状态的舵机固定位置
    private static final double TURRET_MIN = 0.0; // 舵机最小位置
    private static final double TURRET_MAX = 1.0; // 舵机最大位置
    private static final double TURRET_SERVO_RANGE_DEG = 300.0; // 舵机总行程
    private static final double TURRET_MANUAL_STEP_DEG = 2.0; // 每次微调角度
    private static final double TURRET_DEADBAND_DEG = 0.5; // 角度误差死区
    private static final double TURRET_MECH_RATIO = 0.98; // 同步带传动比 147:150
    private static final double TURRET_SIGN = 1.0; // 旋转方向，反向时改为 -1.0
    private static final double TURRET_MOUNT_OFFSET_DEG = 0; // 机械安装偏移角

    private static final double START_X_IN = 72.0; // od 初始 X 坐标
    private static final double START_Y_IN = 72.0; // od 初始 Y 坐标
    private static final double START_HEADING_DEG = 0.0; // od 初始朝向

    private final Servo turretLeft;
    private final Servo turretRight;
    private final GoBildaPinpointDriver pinpoint;
    private final TurretAim turretAim = new TurretAim();

    private double turretPos = TURRET_INIT;
    private double turretTargetYawDeg = START_HEADING_DEG;

    private boolean aimMode = false;
    private boolean lockMode = false;
    private boolean prevLeft, prevRight, prevStart, prevX, prevY;

    private boolean needSetStartPos = true;
    private double pinX = START_X_IN;
    private double pinY = START_Y_IN;
    private double pinHeadingDeg = START_HEADING_DEG;

    private boolean turretReachable = true;
    private double yawErrorDeg = 0.0;
    private double effectiveTargetDeg = 0.0;
    private double turretActualAbsDeg = 0.0;
    private double turretPosRaw = TURRET_INIT;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretLeft = hardwareMap.get(Servo.class, NameConstants.SERVO_LEFT);
        turretRight = hardwareMap.get(Servo.class, NameConstants.SERVO_RIGHT);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, NameConstants.OD);
        setBothPosition(TURRET_INIT);
    }

    public void start() {
        turretPos = TURRET_INIT;
        turretTargetYawDeg = START_HEADING_DEG;
        aimMode = false;
        lockMode = false;
        turretReachable = true;
        needSetStartPos = true;
        turretAim.reset();

        prevLeft = false;
        prevRight = false;
        prevX = false;
        prevY = false;
        prevStart = true;
        setBothPosition(TURRET_INIT);
    }

    /** 每圈调用；X/Y 模式互斥，START 恢复常规模式并重置 od 起点。 */
    public void update(Gamepad gamepad, Telemetry telemetry) {
        updatePinpoint();
        handleStart(gamepad);
        handleModeButtons(gamepad);
        handleManualTrim(gamepad);

        turretAim.update(pinX, pinY);

        if (aimMode) {
            turretAim.updateBearing(pinX, pinY);
            double currentAbsDeg = getCurrentTurretAbsDeg();
            turretTargetYawDeg = turretAim.calculateTargetYawDeg(currentAbsDeg);
            applyTargetAngle();
        } else if (lockMode) {
            applyTargetAngle();
        } else {
            setNormalState();
        }

        setBothPosition(turretPos);

        if (aimMode && !turretReachable) {
            gamepad.rumble(0.8, 0.8, 200);
        } else {
            gamepad.stopRumble();
        }

        addTelemetry(telemetry);
    }

    private void updatePinpoint() {
        pinpoint.update();
        if (needSetStartPos
                && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            resetPinpointPosition();
            needSetStartPos = false;
        }

        Pose2D pose = pinpoint.getPosition();
        pinX = pose.getX(DistanceUnit.INCH);
        pinY = pose.getY(DistanceUnit.INCH);
        pinHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    private void handleModeButtons(Gamepad gamepad) {
        boolean x = gamepad.x;
        if (x && !prevX) {
            aimMode = !aimMode;
            if (aimMode) lockMode = false;
            turretAim.clearDirection();
        }
        prevX = x;

        boolean y = gamepad.y;
        if (y && !prevY) {
            lockMode = !lockMode;
            if (lockMode) {
                aimMode = false;
                turretAim.clearDirection();
                // 以进入锁定瞬间的云台绝对朝向为目标，避免突然甩动。
                turretTargetYawDeg = getCurrentTurretAbsDeg();
            }
        }
        prevY = y;
    }

    private void handleManualTrim(Gamepad gamepad) {
        boolean left = gamepad.dpad_left;
        boolean right = gamepad.dpad_right;

        if (left && !prevLeft) {
            if (aimMode) {
                turretAim.adjustOffset(TURRET_MANUAL_STEP_DEG);
            } else if (lockMode) {
                turretTargetYawDeg = normalizeDeg(
                        turretTargetYawDeg + TURRET_MANUAL_STEP_DEG);
            }
        }
        if (right && !prevRight) {
            if (aimMode) {
                turretAim.adjustOffset(-TURRET_MANUAL_STEP_DEG);
            } else if (lockMode) {
                turretTargetYawDeg = normalizeDeg(
                        turretTargetYawDeg - TURRET_MANUAL_STEP_DEG);
            }
        }
        prevLeft = left;
        prevRight = right;
    }

    /** 将场地绝对目标角转换为舵机位置，使用 od 朝向反向补偿。 */
    private void applyTargetAngle() {
        effectiveTargetDeg = normalizeDeg(
                turretTargetYawDeg + TURRET_MOUNT_OFFSET_DEG);
        yawErrorDeg = normalizeDeg(effectiveTargetDeg - pinHeadingDeg);
        if (Math.abs(yawErrorDeg) < TURRET_DEADBAND_DEG) {
            yawErrorDeg = 0.0;
        }

        double turretRelativeDeg = TURRET_SIGN * yawErrorDeg / TURRET_MECH_RATIO;
        turretPosRaw = TURRET_INIT + turretRelativeDeg / TURRET_SERVO_RANGE_DEG;
        turretReachable = turretPosRaw >= TURRET_MIN && turretPosRaw <= TURRET_MAX;
        turretPos = Range.clip(turretPosRaw, TURRET_MIN, TURRET_MAX);
        turretActualAbsDeg = getCurrentTurretAbsDeg();
    }

    private void setNormalState() {
        turretPos = TURRET_INIT;
        turretPosRaw = TURRET_INIT;
        turretReachable = true;
        yawErrorDeg = 0.0;
        effectiveTargetDeg = 0.0;
        turretActualAbsDeg = getCurrentTurretAbsDeg();
    }

    private double getCurrentTurretAbsDeg() {
        return normalizeDeg(
                pinHeadingDeg
                        + TURRET_MOUNT_OFFSET_DEG
                        + TURRET_SIGN * (turretPos - TURRET_INIT)
                        * TURRET_SERVO_RANGE_DEG / TURRET_MECH_RATIO);
    }

    private void handleStart(Gamepad gamepad) {
        boolean startButton = gamepad.start;
        if (startButton && !prevStart) {
            resetPinpointPosition();
            needSetStartPos = false;
            pinX = START_X_IN;
            pinY = START_Y_IN;
            pinHeadingDeg = START_HEADING_DEG;
            turretTargetYawDeg = START_HEADING_DEG;
            aimMode = false;
            lockMode = false;
            turretAim.reset();
        }
        prevStart = startButton;
    }

    private void resetPinpointPosition() {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, START_X_IN, START_Y_IN,
                AngleUnit.DEGREES, START_HEADING_DEG));
    }

    private void setBothPosition(double position) {
        double safePosition = Range.clip(position, TURRET_MIN, TURRET_MAX);
        turretLeft.setPosition(safePosition);
        turretRight.setPosition(safePosition);
    }

    private void addTelemetry(Telemetry telemetry) {
        String mode = aimMode ? "自瞄 (X 关闭)"
                : lockMode ? "锁定 (Y 关闭)"
                : "常规 0.5 (X 自瞄/Y 锁定)";

        telemetry.addData("Pinpoint 状态", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint X/Y (in)", "(%.2f, %.2f)", pinX, pinY);
        telemetry.addData("Pinpoint Heading (deg)", "%.1f", pinHeadingDeg);
        telemetry.addData("云台状态", mode);
        telemetry.addData("云台舵机位置", "%.3f", turretPos);
        telemetry.addData("云台目标绝对角 (deg)", "%.1f", turretTargetYawDeg);
        telemetry.addData("云台实际绝对角 (deg)", "%.1f", turretActualAbsDeg);
        telemetry.addData("云台锁定误差 (deg)", "%.1f", yawErrorDeg);
        telemetry.addData("云台可达", turretReachable ? "是" : "否（已限幅）");

        if (aimMode) {
            telemetry.addData("当前 Hive 瞄点 (in)", "(%.1f, %.1f)",
                    turretAim.getAimTargetX(), turretAim.getAimTargetY());
            telemetry.addData("自瞄距离 (in)", "%.2f", turretAim.getAimDistance());
            telemetry.addData("指向瞄点绝对角 (deg)", "%.1f", turretAim.getAimTargetDeg());
            telemetry.addData("自瞄偏移角 (deg)", "%.1f", turretAim.getAimOffsetDeg());
            telemetry.addData("有效瞄准已建立", turretAim.hasValidAim() ? "是" : "否");
            if (!turretAim.hasValidAim()) {
                telemetry.addLine("距离瞄点太近，方向未定义，请移开机器人");
            }
        }
    }

    private static double normalizeDeg(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
