package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.mechanisms.DriveSubsystem;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.mechanisms.TurretSubsystem;

@TeleOp(name = "Mecanum Drive", group = "Drive")
public class RobotTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        // 保留进料硬件初始化；进料控制目前未启用。
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        DriveCommand driveCommand = new DriveCommand(drive, gamepad1);

        telemetry.addData("Device status", drive.getDeviceStatus());
        telemetry.addLine("Ready.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        turret.start();
        try {
            while (opModeIsActive()) {
                driveCommand.execute();
                turret.update(gamepad1.y, gamepad1.b, false);
                telemetry.addData("heading deg", "%.1f", drive.getRelativeHeadingDegrees());
                telemetry.addData("rx", "%.3f", gamepad1.right_stick_x);
                telemetry.update();
            }
        } finally {
            drive.stop();
        }
    }
}
