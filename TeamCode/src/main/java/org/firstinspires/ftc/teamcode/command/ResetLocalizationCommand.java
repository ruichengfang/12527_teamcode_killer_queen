package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** Start 键复位 IMU、Pinpoint 起始位置和云台模式。 */
public class ResetLocalizationCommand {
    private final Gamepad gamepad;
    private final LocalizationSubsystem localization;
    private final TurretSubsystem turret;
    private boolean previousStart = true;

    public ResetLocalizationCommand(Gamepad gamepad, LocalizationSubsystem localization,
                                    TurretSubsystem turret) {
        this.gamepad = gamepad;
        this.localization = localization;
        this.turret = turret;
    }

    public void start() {
        previousStart = true;
    }

    public void update() {
        boolean start = gamepad.start;
        if (start && !previousStart) {
            localization.reset();
            turret.reset();
        }
        previousStart = start;
    }
}
