package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

/** 保留 RB > RT > LT 的手动送料优先级。 */
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
        intake.applyManualControl(gamepad.right_bumper, gamepad.right_trigger, gamepad.left_trigger);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
