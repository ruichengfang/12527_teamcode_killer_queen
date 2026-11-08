package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NameConstants;

/** 四轮麦克纳姆底盘。 */
public class DriveSubsystem {
    private final DcMotorEx frontLeft;
    private final DcMotorEx backLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backRight;

    private double frontLeftPower;
    private double backLeftPower;
    private double frontRightPower;
    private double backRightPower;

    public DriveSubsystem(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, NameConstants.FRONT_LEFT);
        backLeft = hardwareMap.get(DcMotorEx.class, NameConstants.BACK_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, NameConstants.FRONT_RIGHT);
        backRight = hardwareMap.get(DcMotorEx.class, NameConstants.BACK_RIGHT);

        configureMotor(frontLeft, DcMotor.Direction.REVERSE);
        configureMotor(backLeft, DcMotor.Direction.REVERSE);
        configureMotor(frontRight, DcMotor.Direction.FORWARD);
        configureMotor(backRight, DcMotor.Direction.FORWARD);
    }

    private static void configureMotor(DcMotorEx motor, DcMotor.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }

    /** 输入均为场地坐标系，forward 正值向前，strafe 正值向右。 */
    public void driveFieldCentric(double forward, double strafe, double turn, double yawRadians) {
        double cos = Math.cos(yawRadians);
        double sin = Math.sin(yawRadians);

        double robotStrafe = strafe * cos + forward * sin;
        double robotForward = -strafe * sin + forward * cos;

        double fl = robotForward + robotStrafe + turn;
        double fr = robotForward - robotStrafe - turn;
        double bl = robotForward - robotStrafe + turn;
        double br = robotForward + robotStrafe - turn;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeftPower = fl;
        backLeftPower = bl;
        frontRightPower = fr;
        backRightPower = br;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    public void stop() {
        driveFieldCentric(0.0, 0.0, 0.0, 0.0);
    }

    public double getFrontLeftPower() {
        return frontLeftPower;
    }

    public double getBackLeftPower() {
        return backLeftPower;
    }

    public double getFrontRightPower() {
        return frontRightPower;
    }

    public double getBackRightPower() {
        return backRightPower;
    }
}
