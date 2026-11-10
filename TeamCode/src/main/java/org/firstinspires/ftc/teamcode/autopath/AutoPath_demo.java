package org.firstinspires.ftc.teamcode.autopath;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.NameConstants;

/** 简易自动：四个底盘电机以 0.7 功率前进 0.2 秒，然后立即刹车停止。 */
@Autonomous(name = "AutoPath_demo", group = "Autonomous")
public class AutoPath_demo extends LinearOpMode {
    private static final double FORWARD_POWER = 0.7;
    private static final long DRIVE_TIME_MS = 200;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, NameConstants.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, NameConstants.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, NameConstants.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, NameConstants.BACK_RIGHT);

        configureMotor(frontLeft, DcMotor.Direction.REVERSE);
        configureMotor(frontRight, DcMotor.Direction.FORWARD);
        configureMotor(backLeft, DcMotor.Direction.REVERSE);
        configureMotor(backRight, DcMotor.Direction.FORWARD);

        telemetry.addLine("准备完成：启动后前进 0.2 秒");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        try {
            setAllPower(FORWARD_POWER);
            sleep(DRIVE_TIME_MS);
        } finally {
            setAllPower(0.0);
        }
    }

    private static void configureMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0.0);
    }

    private void setAllPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
}
