package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;

public class Constants {
    // Based on the existing robot code; confirm directions with Mecanum Tuner.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFrontMotor");
        c.frontRightName.set("rightFrontMotor");
        c.backLeftName.set("leftBackMotor");
        c.backRightName.set("rightBackMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true); // OT drivetrainConfig
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("od");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // 直接引用遥操的实测值，避免两边各写一份后再次漂移。
        c.xPodOffset.set(LocalizationSubsystem.PINPOINT_X_OFFSET_IN);
        c.yPodOffset.set(LocalizationSubsystem.PINPOINT_Y_OFFSET_IN);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // 2026-11-12 14:44 KQ 实车 Foresight Tuner 导出（替换旧值，旧值见下方注释）。
    // 注意：横移最大速度 10.46 in/s、横移自然减速 367.95 明显异常（前进 69.24 / 29.89），
    // 横移那一步可能没测好，若自动横移很慢或乱抖请重跑 Foresight Tuner 的横移部分。
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.49107433874329315);
        Controller secondaryTranslationalForward = Controller.proportional(0.1814388464176489);
        Controller primaryTranslationalLateral = Controller.proportional(2.187894025540989);
        Controller secondaryTranslationalLateral = Controller.proportional(0.80836838083233);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
        c.coast.set(Controller.proportionalFeedforward(0.014091812497175815));
        c.brake.set(Controller.proportionalFeedforward(0.011978040622599443));
        c.headingFeedback.set(Controller.proportional(5.95718587109727));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06885685204964327, 0.005784370933185721));
        c.linearBrakeCoefficients.set(Matrix.diag(0.08425479900727059, 0.03426819968397982));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0025213536722832326, 0.003329157168608347));
        c.maxAchievableForwardVelocity.set(69.24226008529448);
        c.maxAchievableStrafeVelocity.set(10.46290572005562);
        c.naturalForwardDeceleration.set(29.88908823634799);
        c.naturalStrafeDeceleration.set(367.95122926248916);
    });
    /* 旧值（2026-11-12 前）：
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.38393330018585853);
        Controller secondaryTranslationalForward = Controller.proportional(0.14185309553195313);
        Controller primaryTranslationalLateral = Controller.proportional(0.6152812620499513);
        Controller secondaryTranslationalLateral = Controller.proportional(0.22732998570934385);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
        c.coast.set(Controller.proportionalFeedforward(0.013665014127100502));
        c.brake.set(Controller.proportionalFeedforward(0.011615262008035426));
        c.headingFeedback.set(Controller.proportional(5.18755016785564));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06338642014730769, 0.005807550581152556));
        c.linearBrakeCoefficients.set(Matrix.diag(0.14263630430518828, 0.09510941432054501));
        c.quadraticBrakeCoefficients.set(Matrix.diag(9.681199759397902E-4, 0.001749740870689422));
        c.maxAchievableForwardVelocity.set(73.71144044955173);
        c.maxAchievableStrafeVelocity.set(58.59378224521996);
        c.naturalForwardDeceleration.set(31.365307658162813);
        c.naturalStrafeDeceleration.set(62.98938045238759);
    });
    */

    /** Used by the Pedro AutoTune web UI (pedro/Tuning.java, port 10158). */
    public static Foresight createAlgorithm() {
        if (foresightConfig == null) {
            throw new IllegalStateException(
                    "Run AutoTune > Foresight Tuner, then paste its Java export into pedro/Constants.java");
        }
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
}
