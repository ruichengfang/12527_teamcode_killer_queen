package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** X 自瞄、Y 锁定、D-pad 左右微调，并处理不可达震动反馈。 */
public class TurretCommand {
    private final Gamepad gamepad;
    private final TurretSubsystem turret;
    private final LocalizationSubsystem localization;

    private boolean previousX;
    private boolean previousY;
    private boolean previousLeft;
    private boolean previousRight;

    public TurretCommand(Gamepad gamepad, TurretSubsystem turret,
                         LocalizationSubsystem localization) {
        this.gamepad = gamepad;
        this.turret = turret;
        this.localization = localization;
    }

    public void start() {
        previousX = false;
        previousY = false;
        previousLeft = false;
        previousRight = false;
    }

    public void update() {
        handleModeButtons();
        handleAdjustmentButtons();
        turret.update(localization);

        if (turret.isAimMode() && !turret.isReachable()) {
            gamepad.rumble(0.8, 0.8, 200);
        } else {
            gamepad.stopRumble();
        }
    }

    private void handleModeButtons() {
        boolean x = gamepad.x;
        if (x && !previousX) {
            turret.toggleAimMode(localization.isPinpointReady());
        }
        previousX = x;

        boolean y = gamepad.y;
        if (y && !previousY) {
            turret.toggleLockMode(localization.getYawDegrees());
        }
        previousY = y;
    }

    private void handleAdjustmentButtons() {
        boolean left = gamepad.dpad_left;
        boolean right = gamepad.dpad_right;

        if (left && !previousLeft) {
            turret.adjustLeft();
        }
        if (right && !previousRight) {
            turret.adjustRight();
        }

        previousLeft = left;
        previousRight = right;
    }
}
