package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.shoot.ShootConstants;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PathFollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

/**
 * 自动里的一次射击，动作与 Solo 按住 RT 完全相同：选档（舵机 + 转速）→ 飞轮加速，
 * intake 转动 → 达速后开挡板并启动 transit → 送料 FEED_MS 后结束。底盘全程停住。
 * 超时保护：从开始算满 TIMEOUT_MS（默认 5 秒）无论如何都结束，关挡板、停飞轮和送料。
 */
@Config
public class AutoSoloShotCommand extends CommandBase {
    /** transit 开始转动后的送料时间，待实车确认几颗球能送完。 */
    public static long FEED_MS = 1000;
    /** 整次射击的超时保护。 */
    public static long TIMEOUT_MS = 5000;

    private final PathFollowerSubsystem drive;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final BlockerSubsystem blocker;
    private final ShooterSubsystem.ShotMode mode;
    private final ElapsedTime total = new ElapsedTime();
    private final ElapsedTime feed = new ElapsedTime();
    private final ElapsedTime loop = new ElapsedTime();
    private boolean feeding, timedOut;

    public AutoSoloShotCommand(PathFollowerSubsystem drive, ShooterSubsystem shooter,
                               IntakeSubsystem intake, BlockerSubsystem blocker,
                               ShooterSubsystem.ShotMode mode) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.blocker = blocker;
        this.mode = mode;
        addRequirements(drive, shooter, intake, blocker);
    }

    @Override public void initialize() {
        drive.stop();
        shooter.setShotMode(mode);
        shooter.start();
        blocker.setOpen(false);
        feeding = timedOut = false;
        total.reset();
        loop.reset();
    }

    @Override public void execute() {
        drive.stop();
        double dt = Math.max(loop.seconds(), 1e-4);
        loop.reset();
        shooter.update(dt);
        if (!blocker.isOpen() && shooter.isReady()) {
            blocker.setOpen(true);
            feeding = true;
            feed.reset();
        }
        blocker.flushOutput();
        intake.setPowers(ShootConstants.INTAKE_ACTIVE_POWER,
                feeding && feed.seconds() >= ShooterSubsystem.TRANSIT_DELAY_SECONDS
                        ? ShooterSubsystem.SHOT_TRANSIT_POWER : 0.0);
    }

    @Override public boolean isFinished() {
        if (feeding && feed.milliseconds() >= FEED_MS + 1000.0 * ShooterSubsystem.TRANSIT_DELAY_SECONDS) return true;
        timedOut = total.milliseconds() >= TIMEOUT_MS;
        return timedOut;
    }

    @Override public void end(boolean interrupted) {
        blocker.setOpen(false);
        intake.stop();
        shooter.stop();
        shooter.update(1e-4);
        drive.stop();
    }

    public boolean isFeeding() { return feeding; }
    public boolean hasTimedOut() { return timedOut; }
}
