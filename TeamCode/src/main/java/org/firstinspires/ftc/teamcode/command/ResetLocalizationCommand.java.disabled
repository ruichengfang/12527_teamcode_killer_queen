package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 左摇杆按下复位 IMU、Pinpoint 起始位置和云台模式。 */
public class ResetLocalizationCommand extends CommandBase {
    private final LocalizationSubsystem localization;
    private final TurretSubsystem turret;

    public ResetLocalizationCommand(LocalizationSubsystem localization,
                                    TurretSubsystem turret) {
        this.localization = localization;
        this.turret = turret;
        addRequirements(localization, turret);
    }

    @Override
    public void initialize() {
        localization.reset();
        turret.reset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
