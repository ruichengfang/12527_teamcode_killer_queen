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

    public void update() {
        boolean leftBumper = gamepad.left_bumper;
        if (leftBumper && !previousLeftBumper) {
            blocker.toggle();
        }
        previousLeftBumper = leftBumper;
    }
}
