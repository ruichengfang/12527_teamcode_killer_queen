package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;

/**
 * OT DriveCommand：驾驶员相对的无头驾驶（START / 左摇杆按下设定前方）。死区用 KQ 0.05。
 * 2026-11-12：左右摇杆都没有输入时直接锁死电机——四轮功率 0，靠电机 BRAKE（短路制动）停住。
 * 之前用 Pinpoint 拉回位置的做法有回弹，已去掉。
 */
public class DriveCommand extends CommandBase {
    private static final double STICK_DEADZONE = 0.05;

    private final DriveSubsystem drive;
    private final LocalizationSubsystem localization;
    private final DoubleSupplier forward, strafe, turn;
    private boolean braking;

    public DriveCommand(DriveSubsystem drive,
                        LocalizationSubsystem localization,
                        DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        this.drive = drive;
        this.localization = localization;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double f = applyDeadzone(forward.getAsDouble());
        double s = applyDeadzone(strafe.getAsDouble());
        double t = applyDeadzone(turn.getAsDouble());
        braking = f == 0.0 && s == 0.0 && t == 0.0;
        if (braking) {
            drive.stop(); // 功率 0 + ZeroPowerBehavior.BRAKE = 电机锁死
            return;
        }
        drive.moveFieldCentric(f, s, t, localization.getManualDriveHeadingRadians());
    }

    @Override
    public void end(boolean interrupted) {
        braking = false;
        drive.stop();
    }

    public boolean isHolding() {
        return braking;
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) > STICK_DEADZONE ? value : 0.0;
    }
}
