package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.BooleanSupplier;
import org.firstinspires.ftc.teamcode.shoot.ShootConstants;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

/** intake 待机正转；LT 按住吸球，RT 优先射击送料。 */
public final class IntakeCommand extends CommandBase {
    public static final double INTAKE_IDLE_POWER = 0.7;
    private final IntakeSubsystem intake;
    private final Gamepad gamepad;
    private final BooleanSupplier blockerOpen;
    private final BooleanSupplier reverseRequested;
    private boolean intakeEnabled;

    public IntakeCommand(IntakeSubsystem intake, Gamepad gamepad, BooleanSupplier blockerOpen) {
        this(intake, gamepad, blockerOpen, () -> gamepad.left_bumper || gamepad.dpad_down);
    }

    /** 调试 OpMode 可重新分配退球键，以复用相同的吸球和射击送料流程。 */
    public IntakeCommand(IntakeSubsystem intake, Gamepad gamepad, BooleanSupplier blockerOpen,
                         BooleanSupplier reverseRequested) {
        this.intake = intake;
        this.gamepad = gamepad;
        this.blockerOpen = blockerOpen;
        this.reverseRequested = reverseRequested;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intakeEnabled = false;
    }

    @Override
    public void execute() {
        intakeEnabled = gamepad.left_trigger > ShootConstants.TRIGGER_THRESHOLD;
        boolean shootRequested = gamepad.right_trigger > ShootConstants.TRIGGER_THRESHOLD;
        double intakePower = INTAKE_IDLE_POWER;
        double transitPower = 0.0;
        if (shootRequested) {
            boolean feeding = blockerOpen.getAsBoolean();
            intakePower = feeding ? ShootConstants.INTAKE_ACTIVE_POWER : 0.0;
            transitPower = feeding
                    ? org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem.SHOT_TRANSIT_POWER : 0.0;
        } else if (reverseRequested.getAsBoolean()) {
            intakePower = ShootConstants.INTAKE_JAM_POWER;
        } else if (intakeEnabled) {
            intakePower = ShootConstants.INTAKE_ACTIVE_POWER;
            transitPower = ShootConstants.TRANSIT_FEED_POWER;
        }
        intake.setPowers(intakePower, transitPower);
    }

    @Override
    public void end(boolean interrupted) {
        intakeEnabled = false;
        intake.stop();
    }

    public boolean isIntakeEnabled() { return intakeEnabled; }
}
