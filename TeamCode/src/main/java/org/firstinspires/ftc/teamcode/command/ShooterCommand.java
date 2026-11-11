package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

/** 飞轮手动开关与每周期 PIDF 更新；联合发射期间只屏蔽手动开关。 */
public final class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final BooleanSupplier toggle;
    private final DoubleSupplier deltaSeconds;

    public ShooterCommand(ShooterSubsystem shooter, BooleanSupplier toggle,
                          DoubleSupplier deltaSeconds) {
        this.shooter = shooter;
        this.toggle = toggle;
        this.deltaSeconds = deltaSeconds;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (toggle.getAsBoolean()) shooter.toggle();
        shooter.update(deltaSeconds.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.update(1e-4);
    }
}
