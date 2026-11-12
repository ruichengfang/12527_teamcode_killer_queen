package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.shoot.AutoAimController;
import org.firstinspires.ftc.teamcode.shoot.PinpointAimTarget;
import org.firstinspires.ftc.teamcode.shoot.ShootConstants;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;

/** Pinpoint 控制底盘转向，左摇杆继续控制场地坐标平移。 */
public final class ChassisAimCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final LocalizationSubsystem localization;
    private final PinpointAimTarget target;
    private final AutoAimController controller;
    private final Gamepad gamepad;

    public ChassisAimCommand(DriveSubsystem drive, LocalizationSubsystem localization,
                             PinpointAimTarget target, AutoAimController controller, Gamepad gamepad) {
        this.drive = drive;
        this.localization = localization;
        this.target = target;
        this.controller = controller;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        controller.reset();
    }

    @Override
    public void execute() {
        if (target.wasCorrectionApplied()) controller.reset();
        boolean aimValid = target.hasValidAimingTarget() && !target.shouldPauseForAimCalibration();
        double turn = controller.calculateTurnPower(aimValid, target.getYawErrorDegrees());
        // OT ShootAssist：手动平移保持无头（驾驶员参考），转向交给自瞄。
        drive.moveFieldCentric(applyDeadband(-gamepad.left_stick_y),
                applyDeadband(gamepad.left_stick_x), turn, localization.getManualDriveHeadingRadians());
    }

    @Override
    public void end(boolean interrupted) {
        controller.reset();
        drive.stop();
    }

    private static double applyDeadband(double input) {
        return Math.abs(input) > ShootConstants.JOYSTICK_DEADBAND ? input : 0.0;
    }
}
