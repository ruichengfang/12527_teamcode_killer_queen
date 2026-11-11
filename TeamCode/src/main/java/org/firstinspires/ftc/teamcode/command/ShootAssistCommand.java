package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.shoot.AutoAimController;
import org.firstinspires.ftc.teamcode.shoot.PinpointAimTarget;
import org.firstinspires.ftc.teamcode.shoot.PositionHoldController;
import org.firstinspires.ftc.teamcode.shoot.ShootConstants;
import org.firstinspires.ftc.teamcode.shoot.ShotIntake;
import org.firstinspires.ftc.teamcode.shoot.ShotTransit;
import org.firstinspires.ftc.teamcode.shoot.ShotTransit.FiringState;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

/**
 * 移植 Overtune commands/ShootAssistCommand（手动按住射击辅助），逻辑逐行对应 OT。
 * KQ 映射：OT RT → 按住 B；OT LT(吸入) → KQ RB(单独吸入)；OT RB(排堵) → KQ LT(反转)；
 * OT LB(transit 正转) 在 KQ 已用于挡板开关，不绑定。
 * Vision 由 PinpointAimTarget 替代；飞轮 PID 仍由外层每周期调用 ShooterSubsystem.update()。
 */
public class ShootAssistCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final ShotIntake intake;
    private final ShotTransit transit;
    private final PinpointAimTarget vision;
    private final LocalizationSubsystem localization;
    private final AutoAimController autoAim;
    private final PositionHoldController positionHold = new PositionHoldController();
    private final Gamepad gamepad;
    private final boolean autoAimEnabled;

    private boolean canPrepareToFire;
    private boolean manualTranslation;
    private boolean jamActive;

    public ShootAssistCommand(DriveSubsystem drive, ShooterSubsystem shooter, ShotIntake intake,
                              ShotTransit transit, PinpointAimTarget vision,
                              LocalizationSubsystem localization, AutoAimController autoAim,
                              Gamepad gamepad) {
        this(drive, shooter, intake, transit, vision, localization, autoAim, gamepad, true);
    }

    public ShootAssistCommand(DriveSubsystem drive, ShooterSubsystem shooter, ShotIntake intake,
                              ShotTransit transit, PinpointAimTarget vision,
                              LocalizationSubsystem localization, AutoAimController autoAim,
                              Gamepad gamepad, boolean autoAimEnabled) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.transit = transit;
        this.vision = vision;
        this.localization = localization;
        this.autoAim = autoAim;
        this.gamepad = gamepad;
        this.autoAimEnabled = autoAimEnabled;
        addRequirements(drive, shooter, intake.getSubsystem(), transit.getBlocker());
    }

    @Override
    public void initialize() {
        shooter.stop();
        autoAim.reset();
        positionHold.reset();
        manualTranslation = false;
        jamActive = false;
        resetFiringSequence(FiringState.WAITING);
    }

    @Override
    public void execute() {
        // KQ 手柄约定：左摇杆上推为前进（取反），右推为右平移。
        double forward = applyDeadband(-gamepad.left_stick_y);
        double strafe = applyDeadband(gamepad.left_stick_x);
        manualTranslation = forward != 0.0 || strafe != 0.0;
        if (!autoAimEnabled || !vision.isAimCalibrationReady() || manualTranslation) {
            positionHold.reset();
        } else {
            positionHold.update(localization.getFieldPose(), localization.isFieldPoseValid(),
                    localization.getTranslationSpeedInchesPerSecond());
            forward = positionHold.getForwardPower();
            strafe = positionHold.getStrafePower();
        }

        boolean intakeRequested = gamepad.right_bumper;
        boolean jamRequested = gamepad.left_trigger > ShootConstants.TRIGGER_THRESHOLD;
        jamActive = jamRequested;
        intake.updateInputs(intakeRequested, jamRequested, true);
        double turnPower = updateShootRequest(jamRequested);
        // OT LEFT_BUMPER → transit.forwardForIntakeReverse()：KQ LB 已占用，未绑定。
        if (autoAimEnabled && !manualTranslation) {
            drive.driveFieldCentric(forward, strafe, turnPower, 0.0); // OT moveRobot：车体坐标
        } else {
            // 手动平移在射击时保持场地坐标；位置保持输出本身已是车体坐标。
            drive.driveFieldCentric(forward, strafe,
                    autoAimEnabled ? turnPower : applyDeadband(gamepad.right_stick_x),
                    localization.getYawRadians());
        }
    }

    @Override
    public void end(boolean interrupted) {
        autoAim.reset();
        positionHold.reset();
        manualTranslation = false;
        jamActive = false;
        resetFiringSequence(FiringState.IDLE);
        shooter.stop();
        shooter.update(1e-4); // KQ 飞轮无 periodic：与原 AimAndShootCommand 相同，立即写出零功率。
        intake.suspend();
        drive.stop();
    }

    public boolean canFire() {
        return transit.getFiringState() == FiringState.FEEDING;
    }

    public boolean canPrepareToFire() {
        return canPrepareToFire;
    }

    public FiringState getFiringState() {
        return transit.getFiringState();
    }

    public boolean isManualTranslation() {
        return manualTranslation;
    }

    public boolean isPositionHoldLocked() {
        return positionHold.isLocked();
    }

    /** 遥测用：对应 OT TeleOpBase.shootingBlockReason 的判定顺序。 */
    public String getStatus() {
        if (jamActive) return "排堵反转中";
        if (!vision.isAimCalibrationReady()) return "等待 Pinpoint 有效定位";
        if (!vision.hasValidAimingTarget()) return "瞄准目标无效";
        if (transit.getFiringState() == FiringState.FEEDING) return "送料中";
        if (transit.getFiringState() == FiringState.REVERSING) return "已开门，送料前反转中";
        if (!shooter.isReady()) return "飞轮转速尚未稳定";
        if (autoAimEnabled && !autoAim.isLocked()) return "底盘尚未对准";
        if (!shooter.isPitchReady()) return "俯仰未就绪";
        if (!transit.areLimitPositionsCalibrated()) return "挡板位置未校准";
        return "等待射击状态就绪";
    }

    private double updateShootRequest(boolean jamRequested) {
        if (jamRequested) {
            shooter.stop();
            autoAim.reset();
            resetFiringSequence(FiringState.WAITING);
            transit.reverse();
            return 0.0;
        }

        boolean validAimingTarget = vision.hasValidAimingTarget();
        double autoAimTurn = 0.0;
        if (autoAimEnabled) {
            if (vision.wasCorrectionApplied()) autoAim.reset();
            autoAimTurn = autoAim.calculateTurnPower(
                    validAimingTarget && !vision.shouldPauseForAimCalibration(), vision.getYawErrorDegrees());
        }

        double distanceCm = validAimingTarget ? DistanceUnit.INCH.toCm(
                vision.getTargetHorizontalDistanceInches()) : Double.NaN;
        if (!shooter.setDistanceTarget(distanceCm)) {
            // 目标无效也会取消已开始的送料。
            resetFiringSequence(FiringState.WAITING);
            return validAimingTarget ? autoAimTurn : 0.0;
        }

        canPrepareToFire = (!autoAimEnabled || vision.isAimCalibrationReady())
                && shooter.isReady()
                && (!autoAimEnabled || autoAim.isLocked())
                && shooter.isPitchReady();
        transit.requestFeed(canPrepareToFire);

        return validAimingTarget ? autoAimTurn : 0.0;
    }

    private void resetFiringSequence(FiringState resetState) {
        if (transit.getState() == ShotTransit.TransitState.OPEN) {
            intake.clearBallCount();
        }
        canPrepareToFire = false;
        transit.cancelFeeding();
        if (resetState != FiringState.IDLE) {
            transit.requestFeed(false);
        }
    }

    private static double applyDeadband(double input) {
        return Math.abs(input) > ShootConstants.JOYSTICK_DEADBAND ? input : 0.0;
    }
}
