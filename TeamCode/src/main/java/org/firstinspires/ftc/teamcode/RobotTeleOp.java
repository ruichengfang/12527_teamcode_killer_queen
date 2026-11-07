package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

@TeleOp(name = "KillerQueen", group = "Drive")
public class RobotTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Blocker blocker = new Blocker(hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        DriveCommand driveCommand = new DriveCommand(drive, gamepad1);

        telemetry.addData("Device status", drive.getDeviceStatus());
        telemetry.addLine("Ready.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        turret.start();
        shooter.start(getRuntime());
        blocker.start();
        try {
            while (opModeIsActive()) {
                driveCommand.execute();
                turret.update(gamepad1, telemetry);
                shooter.update(gamepad1, getRuntime(), intake, telemetry);
                blocker.update(gamepad1, telemetry);
                telemetry.addData("heading deg", "%.1f", drive.getRelativeHeadingDegrees());
                telemetry.addData("rx", "%.3f", gamepad1.right_stick_x);
                telemetry.update();
            }
        } finally {
            drive.stop();
            shooter.stop();
            intake.stop();
        }
    }
}
