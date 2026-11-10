package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;

/** Gamepad1 双摇杆场地坐标底盘控制。 */
public class DriveCommand {
    private static final double STICK_DEADZONE = 0.05;

    private final DriveSubsystem drive;
    private final LocalizationSubsystem localization;

    public DriveCommand(DriveSubsystem drive,
                        LocalizationSubsystem localization) {
        this.drive = drive;
        this.localization = localization;
    }

    public void update(double forward, double strafe, double turn) {
        drive.driveFieldCentric(applyDeadzone(forward), applyDeadzone(strafe),
                applyDeadzone(turn), localization.getYawRadians());
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }
}
