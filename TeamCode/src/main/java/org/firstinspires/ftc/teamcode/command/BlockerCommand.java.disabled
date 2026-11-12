package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.BooleanSupplier;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;

/** 左 Bumper 切换挡板开合；D-pad 边沿微调位置。 */
public class BlockerCommand extends CommandBase {
    private final Gamepad gamepad;
    private final BlockerSubsystem blocker;
    private boolean previousLeftBumper;

    private final BooleanSupplier increment, decrement;

    public BlockerCommand(Gamepad gamepad, BlockerSubsystem blocker,
                          BooleanSupplier increment, BooleanSupplier decrement) {
        this.gamepad = gamepad;
        this.blocker = blocker;
        this.increment = increment;
        this.decrement = decrement;
        addRequirements(blocker);
    }

    @Override
    public void initialize() {
        previousLeftBumper = false;
    }

    /** 联合发射结束时同步 LB，避免将按住的按钮误当作新按下。 */
    public void syncButtonState() {
        previousLeftBumper = gamepad.left_bumper;
    }

    @Override
    public void execute() {
        boolean leftBumper = gamepad.left_bumper;
        if (leftBumper && !previousLeftBumper) {
            blocker.toggle();
        }
        previousLeftBumper = leftBumper;
        if (increment.getAsBoolean()) blocker.addposition();
        if (decrement.getAsBoolean()) blocker.minposition();
        blocker.flushOutput();
    }
}
