package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

import java.util.function.DoubleSupplier;

/** 按住时启动飞轮并自瞄，满足条件后开启挡板和进料。 */
public class AimAndShootCommand extends CommandBase {
    // 暂定阈值；舵机没有位置反馈，等待时间不能证明云台实际到位。
    private static final double RPM_TOLERANCE = 200.0;
    private static final double RPM_STABLE_SECONDS = 0.30;
    private static final double AIM_STABLE_SECONDS = 0.60;
    private static final double AIM_POSITION_TOLERANCE = 0.01;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final BlockerSubsystem blocker;
    private final IntakeSubsystem intake;
    private final LocalizationSubsystem localization;
    private final DoubleSupplier time;
    private double rpmStableSince;
    private double aimStableSince;
    private double lastAimPosition;
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
        rpmStableSince = Double.NaN;
        aimStableSince = Double.NaN;
        lastAimPosition = Double.NaN;
        blocker.setOpen(false);
        intake.stop();
        shooter.start();
        state = "等待定位";
    }

    @Override
    public void execute() {
        double now = time.getAsDouble();
        boolean localizationReady = localization.isPinpointReady()
                && "READY".equals(localization.getPinpointStatus());
        if (!localizationReady) {
            turret.setAimMode(false, false);
            rpmStableSince = aimStableSince = lastAimPosition = Double.NaN;
            blocker.setOpen(false);
            intake.stop();
            state = "等待 Pinpoint 就绪";
            return;
        }

        turret.setAimMode(true, true);
        boolean aimValid = turret.hasValidAim() && turret.isReachable();
        double target = turret.getPosition();
        if (!aimValid || !Double.isFinite(target)) {
            aimStableSince = lastAimPosition = Double.NaN;
        } else if (Double.isNaN(lastAimPosition)
                || Math.abs(target - lastAimPosition) > AIM_POSITION_TOLERANCE) {
            lastAimPosition = target;
            aimStableSince = now;
        }

        boolean speedReady = shooter.isRunning()
                && Double.isFinite(shooter.getError())
                && Math.abs(shooter.getError()) <= RPM_TOLERANCE;
        if (!speedReady) {
            rpmStableSince = Double.NaN;
        } else if (Double.isNaN(rpmStableSince)) {
            rpmStableSince = now;
        }

        boolean aimSettled = aimValid && !Double.isNaN(aimStableSince)
                && now - aimStableSince >= AIM_STABLE_SECONDS;
        boolean rpmSettled = !Double.isNaN(rpmStableSince)
                && now - rpmStableSince >= RPM_STABLE_SECONDS;
        boolean feed = aimSettled && rpmSettled;
        blocker.setOpen(feed);
        if (feed) {
            intake.setPowers(1.0, 1.0);
            state = "进料中";
        } else {
            intake.stop();
            state = !aimValid ? "等待有效且可达的瞄准角"
                    : !aimSettled ? "等待云台目标稳定" : "等待飞轮转速稳定";
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
