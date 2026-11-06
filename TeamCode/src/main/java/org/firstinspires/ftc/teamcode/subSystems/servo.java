package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.NameConstants;

public class servo {

    // 硬件配置名称（角度来源：od = Pinpoint 定位，不再使用 IMU）
    private static final String SERVO_LEFT_NAME = NameConstants.SERVO_LEFT;
    private static final String SERVO_RIGHT_NAME = NameConstants.SERVO_RIGHT;
    private static final String OD_NAME = NameConstants.OD;

    // 300°舵机参数
    private static final double SERVO_TRAVEL_DEG = 300.0;
    private static final double SERVO_CENTER_DEG = 150.0;

    // 如果自动补偿方向相反，将1.0改成-1.0
    private static final double SERVO_DIRECTION = 1.0;

    // 同步带传动比：147:150 = 0.98（舵机转角 × 0.98 = 云台实际转角）
    // 因此补偿时舵机需要多转一点：补偿角 ÷ 0.98
    private static final double GEAR_RATIO = 147.0 / 150.0;

    private GoBildaPinpointDriver pinpoint;

    // start() 时记录的初始朝向（度），用于计算相对转角
    private double headingOffsetDeg = 0;

    // 使用完整类名，避免与当前servo类混淆
    private com.qualcomm.robotcore.hardware.Servo servoL;
    private com.qualcomm.robotcore.hardware.Servo servoR;

    /**
     * 初始化两个舵机和 Pinpoint。
     * 注意：这里只读取 od 轮反馈的角度，不会调用 resetPosAndIMU，
     * 以免清零底盘正在使用的定位数据。
     */
    public servo(HardwareMap hardwareMap) {

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
    }

    /**
     * 在waitForStart()之后调用。
     * 将正式开始时 od 轮反馈的朝向设为初始方向。
     */
    public void start() {

        pinpoint.update();
        headingOffsetDeg = pinpoint.getHeading(AngleUnit.DEGREES);

        setBothPosition(
                SERVO_CENTER_DEG / SERVO_TRAVEL_DEG
        );
    }

    /**
     * 在MecanumDrive主循环中反复调用。
     *
     * @param toZero 三角键Y：按住时两个舵机转到0
     * @param toOne  圆圈键B：按住时两个舵机转到1
     * @param rset   回中：两个舵机转到0.5（150°中位）
     *
     * 没有按下按键时，执行 od 角度自动反向补偿。
     * 如果两个按键同时按下，toZero优先。
     */
    public void update(boolean toZero, boolean toOne, boolean rset) {

        // 手动控制优先于自动控制
        if (toZero) {
            setBothPosition(0.0);
            return;
        }

        if (toOne) {
            setBothPosition(1.0);
            return;
        }

        if (rset) {
            setBothPosition(0.5);
            return;
        }

        // 读取 od 轮反馈的、相对初始方向的转角
        pinpoint.update();
        double robotYawDeg = normalizeAngle(
                pinpoint.getHeading(AngleUnit.DEGREES) - headingOffsetDeg
        );

        // 舵机向机器人旋转的反方向补偿
        double compensationDeg = -robotYawDeg;

        double rawServoTargetDeg =
                SERVO_CENTER_DEG
                        + SERVO_DIRECTION * compensationDeg / GEAR_RATIO;

        // 限制在300°舵机的机械行程内
        double servoTargetDeg = Range.clip(
                rawServoTargetDeg,
                0.0,
                SERVO_TRAVEL_DEG
        );

        // 将0～300°转换成setPosition需要的0～1
        double servoPosition =
                servoTargetDeg / SERVO_TRAVEL_DEG;

        setBothPosition(servoPosition);
    }

    /**
     * 两个同向舵机接收相同的位置命令。
     */
    private void setBothPosition(double position) {

        double safePosition = Range.clip(
                position,
                0.0,
                1.0
        );

        servoL.setPosition(safePosition);
        servoR.setPosition(safePosition);
    }

    /** 角度归一化到 [-180, 180]，防止 od 累计圈数导致舵机打满。 */
    private double normalizeAngle(double angle) {
        while (angle > 180)   angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}
