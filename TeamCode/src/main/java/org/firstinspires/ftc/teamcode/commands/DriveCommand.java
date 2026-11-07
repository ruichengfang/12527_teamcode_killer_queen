package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

/** 每个控制周期将手柄输入交给底盘子系统。 */
public class DriveCommand {
    private final DriveSubsystem drive;
    private final Gamepad gamepad;

    public DriveCommand(DriveSubsystem drive, Gamepad gamepad) {
        this.drive = drive;
        this.gamepad = gamepad;
    }

    public void execute() {
        drive.updateHeading();
        if (gamepad.left_stick_button) drive.resetFieldHeading();
        drive.driveFieldCentric(gamepad.left_stick_x, -gamepad.left_stick_y,
                gamepad.right_stick_x);
    }
}
