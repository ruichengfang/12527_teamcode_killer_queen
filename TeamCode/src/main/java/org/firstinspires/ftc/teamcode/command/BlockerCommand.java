package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;

/** 左 Bumper 切换 Blocker 开合。 */
public class BlockerCommand {
    private final Gamepad gamepad;
    private final BlockerSubsystem blocker;
    private boolean previousLeftBumper;

    public BlockerCommand(Gamepad gamepad, BlockerSubsystem blocker) {
        this.gamepad = gamepad;
        this.blocker = blocker;
    }

    public void start() {
        previousLeftBumper = false;
    }

    /** 联合发射结束时同步 LB，避免将按住的按钮误当作新按下。 */
    public void syncButtonState() {
        previousLeftBumper = gamepad.left_bumper;
    }

    public void update() {
        boolean leftBumper = gamepad.left_bumper;
        if (leftBumper && !previousLeftBumper) {
            blocker.toggle();
        }
        previousLeftBumper = leftBumper;
    }
}
