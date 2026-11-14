package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

/** 手动模式启动后，飞轮持续跟踪当前目标转速。 */
public final class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier targetRpm;
    private final DoubleSupplier deltaSeconds;

    public ShooterCommand(ShooterSubsystem shooter, DoubleSupplier deltaSeconds) {
        this(shooter, shooter::getShotRpm, deltaSeconds);
    }

    public ShooterCommand(ShooterSubsystem shooter, DoubleSupplier targetRpm, DoubleSupplier deltaSeconds) {
        this.shooter = shooter;
        this.targetRpm = targetRpm;
        this.deltaSeconds = deltaSeconds;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setTargetRpm(targetRpm.getAsDouble());
        shooter.update(deltaSeconds.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.update(1e-4);
    }
}
