package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** Start 键复位 IMU、Pinpoint 起始位置和云台模式。 */
public class ResetLocalizationCommand {
    private final LocalizationSubsystem localization;
    private final TurretSubsystem turret;

    public ResetLocalizationCommand(LocalizationSubsystem localization,
                                    TurretSubsystem turret) {
        this.localization = localization;
        this.turret = turret;
    }

    public void execute() {
        localization.reset();
        turret.reset();
    }
}
