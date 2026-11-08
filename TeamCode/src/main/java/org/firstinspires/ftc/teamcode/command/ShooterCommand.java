package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

/** A 键切换 Shooter，并在每个周期更新转速闭环。 */
public class ShooterCommand {
    private final Gamepad gamepad;
    private final ShooterSubsystem shooter;
    private boolean previousA;

    public ShooterCommand(Gamepad gamepad, ShooterSubsystem shooter) {
        this.gamepad = gamepad;
        this.shooter = shooter;
    }

    public void start() {
        previousA = false;
    }

    public void update(double deltaSeconds) {
        boolean a = gamepad.a;
        if (a && !previousA) {
            shooter.toggle();
        }
        previousA = a;
        shooter.update(deltaSeconds);
    }
}
