package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

import java.util.function.DoubleSupplier;

/** 按住时启动飞轮并尝试自瞄，延时后开启挡板和进料。 */
public class AimAndShootCommand extends CommandBase {
    // 固定等待飞轮加速；实际所需时间需要结合实机测试调整。
    private static final double SHOOTER_SPINUP_SECONDS = 0.1;
    private static final double AIM_TOLERANCE_RADIANS = Math.toRadians(3.0);
    private static final double TURN_POWER_PER_RADIAN = 1.0;
    private static final double MIN_TURN_POWER = 0.07;
    private static final double MAX_TURN_POWER = 0.40;
    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final BlockerSubsystem blocker;
    private final IntakeSubsystem intake;
    private final LocalizationSubsystem localization;
    private final DoubleSupplier time;
    private double shooterStartedAt;
    private String state = "未启动";

    public AimAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter, TurretSubsystem turret,
            BlockerSubsystem blocker, IntakeSubsystem intake,
            LocalizationSubsystem localization, DoubleSupplier time) {
        this.drive = drive;
        this.shooter = shooter;
        this.turret = turret;
        this.blocker = blocker;
        this.intake = intake;
        this.localization = localization;
        this.time = time;
        addRequirements(drive, shooter, turret, blocker, intake);
    }

    @Override
    public void initialize() {
        shooterStartedAt = time.getAsDouble();
        blocker.setOpen(false);
        intake.stop();
        shooter.start();
        state = "等待飞轮加速时间";
    }

    @Override
    public void execute() {
        double now = time.getAsDouble();
        boolean localizationReady = localization.isPinpointReady()
                && localization.isPositionInitialized()
                && "READY".equals(localization.getPinpointStatus());
        double heading = localization.getYawRadians();
        double target = localizationReady ? turret.getChassisAimHeadingRadians(localization) : Double.NaN;
        double difference = target - heading;
        double error = Math.atan2(Math.sin(difference), Math.cos(difference));
        boolean validAim = Double.isFinite(error);
        boolean aligned = validAim && Math.abs(error) <= AIM_TOLERANCE_RADIANS;
        if (validAim && !aligned) {
            double power = Math.max(MIN_TURN_POWER,
                    Math.min(MAX_TURN_POWER, Math.abs(error) * TURN_POWER_PER_RADIAN));
            // Positive drive turn is clockwise, while Pinpoint heading increases CCW.
            drive.driveFieldCentric(0.0, 0.0, -Math.copySign(power, error), 0.0);
        } else {
            drive.stop();
        }

        boolean feed = aligned && now - shooterStartedAt >= SHOOTER_SPINUP_SECONDS;
        blocker.setOpen(feed);
        if (feed) {
            intake.setPowers(1.0, 1.0);
            state = "进料中";
        } else {
            intake.stop();
            if (!validAim) state = "等待 Pinpoint 有效定位";
            else if (!aligned) state = "底盘正在瞄准";
            else state = "等待飞轮加速时间";
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        intake.stop();
        blocker.setOpen(false);
        shooter.stop();
        shooter.update(1e-4);
        state = "已停止";
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public String getState() {
        return state;
    }
}
