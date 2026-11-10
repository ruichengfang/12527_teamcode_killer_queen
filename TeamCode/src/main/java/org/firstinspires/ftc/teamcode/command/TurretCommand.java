package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 协调云台和定位；按键及手柄反馈由主程序处理。 */
public class TurretCommand {
    private final TurretSubsystem turret;
    private final LocalizationSubsystem localization;

    public TurretCommand(TurretSubsystem turret,
                         LocalizationSubsystem localization) {
        this.turret = turret;
        this.localization = localization;
    }

    public void update(boolean toggleAim, boolean toggleLock, boolean left, boolean right) {
        if (toggleAim) {
            turret.toggleAimMode(localization.isPinpointReady());
        }
        if (toggleLock) {
            turret.toggleLockMode(localization.getYawDegrees());
        }
        if (left) {
            turret.adjustLeft();
        }
        if (right) {
            turret.adjustRight();
        }
        turret.update(localization);
    }
}
