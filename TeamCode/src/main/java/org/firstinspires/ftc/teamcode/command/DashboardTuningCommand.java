package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.DashboardTuningSubsystem;

/** 每周期应用 Dashboard 的诊断目标，退出时停止电机。 */
public final class DashboardTuningCommand extends CommandBase {
    private final DashboardTuningSubsystem tuning;

    public DashboardTuningCommand(DashboardTuningSubsystem tuning) {
        this.tuning = tuning;
        addRequirements(tuning);
    }

    @Override
    public void execute() {
        tuning.updateOutputs();
    }

    @Override
    public void end(boolean interrupted) {
        tuning.stopMotors();
    }
}
