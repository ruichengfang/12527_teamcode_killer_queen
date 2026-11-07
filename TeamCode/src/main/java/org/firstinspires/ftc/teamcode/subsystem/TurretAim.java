package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.NameConstants;

/**
 * 云台自瞄：完全基于 od（Pinpoint）定位。
 *
 * 每周期读取车体在场地中的坐标 (x, y) 和朝向 heading，
 * 计算场地中心相对车头的角度，驱动两个 300° 舵机，
 * 使云台始终朝向场地中心 (72, 72)。
 *
 * 坐标约定：英寸，0~144；heading 逆时针为正，0° 朝 +x 方向。
 * 舵机中位 0.5（150°）= 云台朝车头正前方。
 *
 * 注意：主程序必须在 init 时用 pinpoint.setPosition() 设置
 * 真实的起始场地坐标，否则解算出的方向不准。
 */
public class TurretAim {

    private static final String SERVO_LEFT_NAME = NameConstants.SERVO_LEFT;
    private static final String SERVO_RIGHT_NAME = NameConstants.SERVO_RIGHT;
    private static final String OD_NAME = NameConstants.OD;

    // 场地中心坐标（英寸）
    private static final double CENTER_X = 72.0;
    private static final double CENTER_Y = 72.0;

    // 300°舵机参数
    private static final double SERVO_TRAVEL_DEG = 300.0;
    private static final double SERVO_CENTER_DEG = 150.0;

    // 如果云台转向与预期相反，将 1.0 改成 -1.0
    private static final double SERVO_DIRECTION = 1.0;

    private final GoBildaPinpointDriver pinpoint;
    private final com.qualcomm.robotcore.hardware.Servo servoL;
    private final com.qualcomm.robotcore.hardware.Servo servoR;

    // 遥测用
    private double targetFieldDeg = 0;
    private double relativeDeg = 0;
    private double servoPosition = 0.5;

    public TurretAim(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, OD_NAME);
        servoL = hardwareMap.get(
                com.qualcomm.robotcore.hardware.Servo.class, SERVO_LEFT_NAME);
        servoR = hardwareMap.get(
                com.qualcomm.robotcore.hardware.Servo.class, SERVO_RIGHT_NAME);
        setBoth(SERVO_CENTER_DEG / SERVO_TRAVEL_DEG);
    }

    /** 每周期调用：根据 od 坐标与朝向，让云台指向场地中心。 */
    public void update() {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();
        double x = pos.getX(DistanceUnit.INCH);
        double y = pos.getY(DistanceUnit.INCH);
        double headingDeg = pos.getHeading(AngleUnit.DEGREES);

        // 场地中心相对车体的方位角（场地方向）
        targetFieldDeg = Math.toDegrees(Math.atan2(CENTER_Y - y, CENTER_X - x));

        // 转成相对车头的角度，再映射到舵机行程
        relativeDeg = normalizeAngle(targetFieldDeg - headingDeg);
        double servoDeg = Range.clip(
                SERVO_CENTER_DEG + SERVO_DIRECTION * relativeDeg,
                0.0, SERVO_TRAVEL_DEG);
        servoPosition = servoDeg / SERVO_TRAVEL_DEG;
        setBoth(servoPosition);
    }

    public double getTargetFieldDeg() { return targetFieldDeg; }
    public double getRelativeDeg()    { return relativeDeg; }
    public double getServoPosition()  { return servoPosition; }

    private void setBoth(double position) {
        double p = Range.clip(position, 0.0, 1.0);
        servoL.setPosition(p);
        servoR.setPosition(p);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180)   angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}
