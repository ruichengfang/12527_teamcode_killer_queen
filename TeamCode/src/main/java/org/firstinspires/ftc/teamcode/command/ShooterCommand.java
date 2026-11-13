package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

/** 自瞄模式常转；贴墙模式在待机和射击转速之间切换。 */
public final class ShooterCommand extends CommandBase {
    public static final double WALL_STANDBY_RPM = 2000.0;
    public static final double WALL_SHOT_RPM = 3450.0;
    private final ShooterSubsystem shooter;
    private final BooleanSupplier wallMode, shotRequested;
    private final DoubleSupplier deltaSeconds;

    public ShooterCommand(ShooterSubsystem shooter, BooleanSupplier wallMode, BooleanSupplier shotRequested,
                          DoubleSupplier deltaSeconds) {
        this.shooter = shooter;
        this.wallMode = wallMode;
        this.shotRequested = shotRequested;
        this.deltaSeconds = deltaSeconds;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setTargetRpm(wallMode.getAsBoolean()
                ? (shotRequested.getAsBoolean() ? WALL_SHOT_RPM : WALL_STANDBY_RPM)
                : shooter.getShotRpm());
        shooter.update(deltaSeconds.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.update(1e-4);
    }
}
