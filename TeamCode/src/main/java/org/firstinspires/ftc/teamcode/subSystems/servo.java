package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.NameConstants;

public class servo {

    // 硬件配置名称
    private static final String SERVO_LEFT_NAME = NameConstants.SERVO_LEFT;
    private static final String SERVO_RIGHT_NAME = NameConstants.SERVO_RIGHT;
    private static final String IMU_NAME = NameConstants.IMU;

    // 300°舵机参数
    private static final double SERVO_TRAVEL_DEG = 300.0;
    private static final double SERVO_CENTER_DEG = 150.0;

    // 如果自动补偿方向相反，将1.0改成-1.0
    private static final double SERVO_DIRECTION = 1.0;

    private IMU imu;

    // 使用完整类名，避免与当前servo类混淆
    private com.qualcomm.robotcore.hardware.Servo servoL;
    private com.qualcomm.robotcore.hardware.Servo servoR;

    /**
     * 初始化两个舵机和Control Hub内置IMU。
     */
    public servo(HardwareMap hardwareMap) {

        imu = hardwareMap.get(IMU.class, IMU_NAME);

        servoL = hardwareMap.get(
                com.qualcomm.robotcore.hardware.Servo.class,
                SERVO_LEFT_NAME
        );

        servoR = hardwareMap.get(
                com.qualcomm.robotcore.hardware.Servo.class,
                SERVO_RIGHT_NAME
        );

        /*
         * 当前设置适用于：
         * Control Hub平放；
         * Logo朝上；
         * USB接口朝机器人前方。
         *
         * 如果Control Hub安装方向不同，需要修改这两个方向。
         */
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // 初始化时，让两个舵机来到150°中位
        setBothPosition(
                SERVO_CENTER_DEG / SERVO_TRAVEL_DEG
        );
    }

    /**
     * 在waitForStart()之后调用。
     * 将正式开始时的机器人方向设为初始方向。
     */
    public void start() {

        imu.resetYaw();

        setBothPosition(
                SERVO_CENTER_DEG / SERVO_TRAVEL_DEG
        );
    }

    /**
     * 在MecanumDrive主循环中反复调用。
     *
     * @param toZero 三角键Y：按住时两个舵机转到0
     * @param toOne  圆圈键B：按住时两个舵机转到1
     *
     * 没有按下按键时，执行IMU自动反向补偿。
     * 如果两个按键同时按下，toZero优先。
     */
    public void update(boolean toZero, boolean toOne, boolean rset) {

        // 手动控制优先于IMU自动控制
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

        // 读取机器人相对初始方向的Yaw角
        YawPitchRollAngles orientation =
                imu.getRobotYawPitchRollAngles();

        double robotYawDeg =
                orientation.getYaw(AngleUnit.DEGREES);

        // 舵机向机器人旋转的反方向补偿
        double compensationDeg = -robotYawDeg;

        double rawServoTargetDeg =
                SERVO_CENTER_DEG
                        + SERVO_DIRECTION * compensationDeg;

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
}