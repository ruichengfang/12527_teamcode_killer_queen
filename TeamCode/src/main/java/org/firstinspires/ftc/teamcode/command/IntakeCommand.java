package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

/** RB 单独吸入，RT 正向联动，LT 反向联动。 */
public class IntakeCommand {
    private static final double TRIGGER_THRESHOLD = 0.05;
    private static final double INTAKE_SOLO_POWER = 1.0;

    private final Gamepad gamepad;
    private final IntakeSubsystem intake;

    public IntakeCommand(Gamepad gamepad, IntakeSubsystem intake) {
        this.gamepad = gamepad;
        this.intake = intake;
    }

    public void update() {
        if (gamepad.right_bumper) {
            intake.setPowers(INTAKE_SOLO_POWER, 0.0);
            return;
        }

        double rightTrigger = gamepad.right_trigger;
        double leftTrigger = gamepad.left_trigger;
        if (rightTrigger > TRIGGER_THRESHOLD) {
            intake.setPowers(rightTrigger, rightTrigger);
        } else if (leftTrigger > TRIGGER_THRESHOLD) {
            intake.setPowers(-leftTrigger, -leftTrigger);
        } else {
            intake.stop();
        }
    }
}
