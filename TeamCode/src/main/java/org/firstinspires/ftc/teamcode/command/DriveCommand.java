package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;

/** Gamepad1 双摇杆场地坐标底盘控制。 */
public class DriveCommand {
    private static final double STICK_DEADZONE = 0.05;

    private final Gamepad gamepad;
    private final DriveSubsystem drive;
    private final LocalizationSubsystem localization;

    public DriveCommand(Gamepad gamepad, DriveSubsystem drive,
                        LocalizationSubsystem localization) {
        this.gamepad = gamepad;
        this.drive = drive;
        this.localization = localization;
    }

    public void update() {
        double forward = applyDeadzone(-gamepad.left_stick_y);
        double strafe = applyDeadzone(gamepad.left_stick_x);
        double turn = applyDeadzone(gamepad.right_stick_x);
        drive.driveFieldCentric(forward, strafe, turn, localization.getYawRadians());
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }
}
