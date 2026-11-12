package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.BooleanSupplier;
import org.firstinspires.ftc.teamcode.shoot.ShootConstants;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

/** RT 射击时等待挡板打开再启动 transit；LT 吸入和 LB 退球独立控制。 */
public final class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final Gamepad gamepad;
    private final BooleanSupplier blockerOpen;

    public IntakeCommand(IntakeSubsystem intake, Gamepad gamepad, BooleanSupplier blockerOpen) {
        this.intake = intake;
        this.gamepad = gamepad;
        this.blockerOpen = blockerOpen;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        boolean intakeRequested = gamepad.left_trigger > ShootConstants.TRIGGER_THRESHOLD;
        boolean shootRequested = gamepad.right_trigger > ShootConstants.TRIGGER_THRESHOLD;
        double intakePower = 0.0;
        double transitPower = 0.0;
        if (gamepad.left_bumper && !shootRequested) {
            // 单独按住 LB 时全速退球；LB + RT 保留原有近射组合键。
            intakePower = ShootConstants.INTAKE_JAM_POWER;
            transitPower = ShootConstants.TRANSIT_REVERSE_POWER;
        } else if (gamepad.dpad_down) {
            intakePower = ShootConstants.INTAKE_JAM_POWER;
        } else {
            if (intakeRequested) {
                intakePower = ShootConstants.INTAKE_ACTIVE_POWER;
                transitPower = ShootConstants.TRANSIT_FEED_POWER;
            }
            if (shootRequested) {
                intakePower = ShootConstants.INTAKE_ACTIVE_POWER;
                transitPower = blockerOpen.getAsBoolean()
                        ? org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem.SHOT_TRANSIT_POWER : 0.0;
            }
        }
        intake.setPowers(intakePower, transitPower);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
