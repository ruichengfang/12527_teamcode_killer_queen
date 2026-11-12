package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NameConstants;

/** 四轮麦克纳姆底盘。 */
public class DriveSubsystem extends SubsystemBase {
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

    /**
     * OT Drive.moveFieldCentric（逻辑照抄，电机方向仍用 KQ 自己的）。
     * driverHeadingRadians 为 NaN 时退回车头坐标驾驶；输入非有限值时停车。
     */
    public void moveFieldCentric(double forward, double strafe, double turn, double driverHeadingRadians) {
        if (!Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(turn)) {
            stop();
            return;
        }
        double heading = driverHeadingRadians;
        if (!Double.isFinite(heading)) {
            moveRobot(forward, strafe, turn);
            return;
        }
        // Field heading is CCW-positive; joystick strafe is right-positive.
        double robotForward = forward * Math.cos(heading) - strafe * Math.sin(heading);
        double robotStrafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        moveRobot(robotForward, robotStrafe, turn);
    }

    /** OT Drive.moveRobot：总和归一化混控。 */
    public void moveRobot(double forward, double strafe, double turn) {
        double rotX = strafe;
        double rotY = forward;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        setMotorPowers((rotY + rotX + turn) / denominator, (rotY - rotX + turn) / denominator,
                (rotY - rotX - turn) / denominator, (rotY + rotX - turn) / denominator);
    }

    private void setMotorPowers(double fl, double bl, double fr, double br) {
        frontLeftPower = fl;
        backLeftPower = bl;
        frontRightPower = fr;
        backRightPower = br;
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    /** 旧版 KQ 场地坐标驾驶（最大轮归一化），仅旧测试/旧命令使用，Solo 不再调用。 */
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
        moveRobot(0, 0, 0);
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
