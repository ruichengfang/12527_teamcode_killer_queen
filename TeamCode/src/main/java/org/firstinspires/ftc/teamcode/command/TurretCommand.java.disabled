package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 协调云台和定位；按键边沿及手柄反馈由手动组合命令处理。 */
public class TurretCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final LocalizationSubsystem localization;

    private final BooleanSupplier toggleAim, toggleLock, left, right;

    public TurretCommand(TurretSubsystem turret,
                         LocalizationSubsystem localization,
                         BooleanSupplier toggleAim, BooleanSupplier toggleLock,
                         BooleanSupplier left, BooleanSupplier right) {
        this.turret = turret;
        this.localization = localization;
        this.toggleAim = toggleAim;
        this.toggleLock = toggleLock;
        this.left = left;
        this.right = right;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (toggleAim.getAsBoolean()) {
            turret.toggleAimMode(localization.isPinpointReady());
        }
        if (toggleLock.getAsBoolean()) {
            turret.toggleLockMode(localization.getYawDegrees());
        }
        if (left.getAsBoolean()) {
            turret.adjustLeft();
        }
        if (right.getAsBoolean()) {
            turret.adjustRight();
        }
        turret.update(localization);
    }
}
