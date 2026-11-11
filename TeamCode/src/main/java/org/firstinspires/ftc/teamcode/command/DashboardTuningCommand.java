package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.DashboardTuner;

/** Applies live Dashboard targets to the isolated tuning hardware. */
public class DashboardTuningCommand extends CommandBase {
    private final DashboardTuner tuner;

    public DashboardTuningCommand(DashboardTuner tuner) {
        this.tuner = tuner;
        addRequirements(tuner);
    }

    @Override
    public void execute() { tuner.applyTargets(); }

    @Override
    public void end(boolean interrupted) { tuner.stop(); }
}
