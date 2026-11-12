package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.shoot.ShootConstants;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

/** LT 或 RT（射击）：intake + transit 全速；方向键下优先反转吐球。 */
public final class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final Gamepad gamepad;

    public IntakeCommand(IntakeSubsystem intake, Gamepad gamepad) {
        this.intake = intake;
        this.gamepad = gamepad;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        boolean intakeRequested = gamepad.left_trigger > ShootConstants.TRIGGER_THRESHOLD;
        // 2026-11-12：射击（按住 RT）时改为 transitMotor 全速送料，intake 不再随 RT 转。
        boolean shootRequested = gamepad.right_trigger > ShootConstants.TRIGGER_THRESHOLD;
        double intakePower = 0.0;
        double transitPower = 0.0;
        if (gamepad.dpad_down) {
            intakePower = -1.0;
        } else {
            // 2026-11-12：LT 吸球时 intake 与 transit 都满功率；RT 射击时 transit 满功率。
            if (intakeRequested) {
                intakePower = ShootConstants.INTAKE_ACTIVE_POWER;
                transitPower = ShootConstants.TRANSIT_FEED_POWER;
            }
            // 2026-11-12：RT 准备射击时 intake 也满功率转。
            if (shootRequested) {
                intakePower = ShootConstants.INTAKE_ACTIVE_POWER;
                transitPower = ShootConstants.TRANSIT_FEED_POWER;
            }
        }
        intake.setPowers(intakePower, transitPower);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
