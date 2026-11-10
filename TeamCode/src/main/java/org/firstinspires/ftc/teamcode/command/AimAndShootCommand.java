package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

import java.util.function.DoubleSupplier;

/** 按住时启动飞轮并尝试自瞄，延时后开启挡板和进料。 */
public class AimAndShootCommand extends CommandBase {
    // 固定等待飞轮加速；实际所需时间需要结合实机测试调整。
    private static final double SHOOTER_SPINUP_SECONDS = 1;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final BlockerSubsystem blocker;
    private final IntakeSubsystem intake;
    private final LocalizationSubsystem localization;
    private final DoubleSupplier time;
    private double shooterStartedAt;
    private String state = "未启动";

    public AimAndShootCommand(ShooterSubsystem shooter, TurretSubsystem turret,
            BlockerSubsystem blocker, IntakeSubsystem intake,
            LocalizationSubsystem localization, DoubleSupplier time) {
        this.shooter = shooter;
        this.turret = turret;
        this.blocker = blocker;
        this.intake = intake;
        this.localization = localization;
        this.time = time;
        addRequirements(shooter, turret, blocker, intake);
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
                && "READY".equals(localization.getPinpointStatus());
        turret.setAimMode(localizationReady, localizationReady);
        boolean feed = now - shooterStartedAt >= SHOOTER_SPINUP_SECONDS;
        blocker.setOpen(feed);
        if (feed) {
            intake.setPowers(1.0, 1.0);
            state = localizationReady ? "进料中" : "进料中（Pinpoint 未就绪）";
        } else {
            intake.stop();
            state = "等待飞轮加速时间";
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        blocker.setOpen(false);
        shooter.stop();
        shooter.update(1e-4);
        turret.setAimMode(false, false);
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
