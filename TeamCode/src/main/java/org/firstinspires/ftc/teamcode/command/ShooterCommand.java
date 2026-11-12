package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

/** 按给定条件运行飞轮，并每周期更新转速闭环。 */
public final class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final BooleanSupplier triggerHeld;
    private final DoubleSupplier deltaSeconds;

    public ShooterCommand(ShooterSubsystem shooter, BooleanSupplier triggerHeld,
                          DoubleSupplier deltaSeconds) {
        this.shooter = shooter;
        this.triggerHeld = triggerHeld;
        this.deltaSeconds = deltaSeconds;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (triggerHeld.getAsBoolean()) {
            shooter.start();
        } else {
            shooter.stop();
        }
        shooter.update(deltaSeconds.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.update(1e-4);
    }
}
