package org.firstinspires.ftc.teamcode.autopath;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AutoEndPose;
import org.firstinspires.ftc.teamcode.NameConstants;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;

/** 简易自动：四个底盘电机以 0.7 功率前进 0.2 秒，然后立即刹车停止。 */
@Autonomous(name = "AutoPath_demo", group = "Autonomous")
public class AutoPath_demo extends LinearOpMode {
    private static final double FORWARD_POWER = 0.7;
    private static final long DRIVE_TIME_MS = 200;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private LocalizationSubsystem localization;

    @Override
    public void runOpMode() {
        AutoEndPose.clear();
        frontLeft = hardwareMap.get(DcMotor.class, NameConstants.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, NameConstants.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, NameConstants.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, NameConstants.BACK_RIGHT);

        configureMotor(frontLeft, DcMotor.Direction.REVERSE);
        configureMotor(frontRight, DcMotor.Direction.FORWARD);
        configureMotor(backLeft, DcMotor.Direction.REVERSE);
        configureMotor(backRight, DcMotor.Direction.FORWARD);

        // 简易自动固定从自己的起点开始，不读取上一场自动的末位置。
        localization = new LocalizationSubsystem(hardwareMap,
                LocalizationSubsystem.START_X_IN,
                LocalizationSubsystem.START_Y_IN,
                Math.toRadians(LocalizationSubsystem.START_HEADING_DEG));

        while (!isStarted() && !isStopRequested()) {
            localization.update();
            showPose("等待开始；启动后前进 0.2 秒");
            sleep(20);
        }
        if (isStopRequested()) {
            return;
        }

        try {
            setAllPower(FORWARD_POWER);
            long driveEndMs = System.currentTimeMillis() + DRIVE_TIME_MS;
            while (opModeIsActive() && System.currentTimeMillis() < driveEndMs) {
                localization.update();
                saveCurrentPose();
                showPose("前进中");
                sleep(5);
            }
        } finally {
            setAllPower(0.0);
            localization.update();
            saveCurrentPose();
        }
    }

    private void saveCurrentPose() {
        if (localization.isPositionInitialized()) {
            AutoEndPose.save(localization.getXIn(), localization.getYIn(),
                    Math.toRadians(localization.getHeadingDeg()));
        }
    }

    private void showPose(String state) {
        telemetry.addData("状态", state);
        telemetry.addData("Pinpoint", localization.getPinpointStatus());
        if (localization.isPositionInitialized()) {
            telemetry.addData("x (in)", localization.getXIn());
            telemetry.addData("y (in)", localization.getYIn());
            telemetry.addData("heading (deg)", localization.getHeadingDeg());
            telemetry.addData("坐标遗传", AutoEndPose.hasValue() ? "当前位置已保存" : "等待有效定位");
        } else {
            telemetry.addLine("等待 Pinpoint READY 并写入起点");
        }
        telemetry.update();
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
