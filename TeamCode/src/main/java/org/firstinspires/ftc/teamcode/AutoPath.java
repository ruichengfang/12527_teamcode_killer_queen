package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

/**
 * 四点自动路线，无 Pedro Pathing。
 *
 * 1. 从 (58, 8) 向前移动到射击点 (58, 23)；
 * 2. 朝 (58, 52) 的 Hive 射击；
 * 3. 向前移动到 (58, 108)；
 * 4. 向左平移到 (14, 108) 并停止。
 *
 * 坐标单位为英寸，0° 朝 +X，逆时针为正，因此前进 +Y 对应 90°。
 */
@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    /* ==================== 四个路径点 ==================== */
    private static final double[] START = {58.0, 8.0, 90.0};
    private static final double[] SHOOT_POINT = {58.0, 23.0, 90.0};
    private static final double[] POINT2 = {58.0, 108.0, 90.0};
    private static final double[] END_POINT = {14.0, 108.0, 90.0};

    /* ==================== Hive 坐标 ==================== */
    private static final double[] HIVE = {58.0, 52.0};

    /* ==================== 射击参数 ==================== */
    private static final double SHOOT_RPM = 3500; // 飞轮目标转速
    private static final double RPM_READY_RATIO = 0.95; // 达到目标转速 95% 后供弹
    private static final double SPINUP_TIMEOUT_S = 4.0; // 转速等待超时
    private static final double FEED_TIME_S = 3.0; // 供弹持续时间
    private static final double AIM_TOLERANCE_DEG = 2.0; // 射击对准容差
    private static final double AIM_TIMEOUT_S = 3.0; // 对准超时

    /* ==================== 走点控制参数 ==================== */
    private static final double DRIVE_KP = 0.08; // 每英寸位置误差对应的功率
    private static final double MAX_DRIVE_POWER = 0.7; // 平移最大功率
    private static final double MIN_DRIVE_POWER = 0.15; // 克服静摩擦的最小平移功率
    private static final double TURN_KP = 0.01; // 朝向误差比例系数
    private static final double MAX_TURN_POWER = 0.6; // 转向最大功率
    private static final double MIN_TURN_POWER = 0.08; // 克服静摩擦的最小转向功率
    private static final double X_FACTOR = 1.1; // 与遥控底盘一致的横移补偿
    private static final double POS_TOLERANCE = 1.0; // 位置到位容差，单位英寸
    private static final double HEAD_TOLERANCE_DEG = 3.0; // 走点朝向容差
    private static final double GO_TO_TIMEOUT_S = 8.0; // 每段走点超时

    private DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    private GoBildaPinpointDriver pinpoint;
    private Shooter shooter;
    private IntakeSubsystem intake;

    @Override
    public void runOpMode() {
        leftFrontMotor = hardwareMap.get(DcMotor.class, NameConstants.LEFT_FRONT_MOTOR);
        rightFrontMotor = hardwareMap.get(DcMotor.class, NameConstants.RIGHT_FRONT_MOTOR);
        leftBackMotor = hardwareMap.get(DcMotor.class, NameConstants.LEFT_BACK_MOTOR);
        rightBackMotor = hardwareMap.get(DcMotor.class, NameConstants.RIGHT_BACK_MOTOR);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        // 与当前 DriveSubsystem 使用相同的 Pinpoint 配置。
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, NameConstants.OD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(-20, -135.2, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
        pinpoint.update();
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, START[0], START[1], AngleUnit.DEGREES, START[2]));
        pinpoint.update();

        shooter = new Shooter(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        telemetry.addLine("Ready: (58,8) -> (58,23) -> shoot -> (58,108) -> (14,108)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        try {
            goTo(SHOOT_POINT);
            if (opModeIsActive()) shootAtHive();
            if (opModeIsActive()) goTo(POINT2);
            if (opModeIsActive()) goTo(END_POINT);
        } finally {
            stopDrive();
            shooter.stop();
            intake.stop();
        }
    }

    /** 闭环移动到目标点 (x, y, heading)，带位置和朝向到位判断。 */
    private void goTo(double[] target) {
        double startTime = getRuntime();

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double headingDeg = pose.getHeading(AngleUnit.DEGREES);

            double dx = target[0] - x;
            double dy = target[1] - y;
            double distance = Math.hypot(dx, dy);
            double headingError = angleErrorDeg(target[2], headingDeg);

            if (distance < POS_TOLERANCE
                    && Math.abs(headingError) < HEAD_TOLERANCE_DEG) {
                break;
            }
            if (getRuntime() - startTime > GO_TO_TIMEOUT_S) {
                telemetry.addLine("走点超时，停止本段并继续后续流程");
                break;
            }

            // 位置已到位时只修正朝向，避免在目标点附近继续漂移。
            double drivePower = 0.0;
            if (distance > POS_TOLERANCE) {
                drivePower = Math.min(MAX_DRIVE_POWER, distance * DRIVE_KP);
                drivePower = Math.max(MIN_DRIVE_POWER, drivePower);
            }

            double direction = Math.atan2(dy, dx) - Math.toRadians(headingDeg);
            double xPower = drivePower * Math.cos(direction) * X_FACTOR;
            double yPower = drivePower * Math.sin(direction);
            double turnPower = headingControl(target[2], headingDeg);

            setDrive(xPower, yPower, turnPower);

            telemetry.addData("阶段", "goTo");
            telemetry.addData("目标点", "(%.1f, %.1f, %.1f°)",
                    target[0], target[1], target[2]);
            telemetry.addData("当前位置", "(%.1f, %.1f, %.1f°)", x, y, headingDeg);
            telemetry.addData("距离目标", "%.2f in", distance);
            telemetry.addData("朝向误差", "%.2f deg", headingError);
            telemetry.update();
        }
        stopDrive();
    }

    /** 在射击点对准固定 Hive，飞轮达到转速后供弹。 */
    private void shootAtHive() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double targetDeg = Math.toDegrees(Math.atan2(HIVE[1] - y, HIVE[0] - x));

        // 原地对准，同时逐渐提高飞轮目标转速。
        double startTime = getRuntime();
        while (opModeIsActive()) {
            spinUpTick();
            double error = holdAim(targetDeg);
            if (Math.abs(error) < AIM_TOLERANCE_DEG) break;
            if (getRuntime() - startTime > AIM_TIMEOUT_S) break;
        }

        // 保持对准，等待飞轮达到射击转速。
        startTime = getRuntime();
        while (opModeIsActive()
                && shooter.getFilteredRPM() < SHOOT_RPM * RPM_READY_RATIO
                && getRuntime() - startTime < SPINUP_TIMEOUT_S) {
            spinUpTick();
            holdAim(targetDeg);
        }

        // 供弹 2/3 正转，intake/transit 联动送球。
        startTime = getRuntime();
        while (opModeIsActive() && getRuntime() - startTime < FEED_TIME_S) {
            shooter.update(getRuntime(), false, false, false, true, false, 0, 0);
            intake.intake();
            holdAim(targetDeg);

            telemetry.addData("阶段", "shoot");
            telemetry.addData("Hive", "(%.1f, %.1f)", HIVE[0], HIVE[1]);
            telemetry.addData("目标角度", "%.1f", targetDeg);
            telemetry.addData("飞轮 RPM", "%.0f", shooter.getFilteredRPM());
            telemetry.update();
        }

        shooter.stop();
        intake.stop();
        stopDrive();
    }

    /** 模拟十字键上下，让目标转速平稳升至 SHOOT_RPM。 */
    private void spinUpTick() {
        double target = shooter.getTargetRPM();
        boolean up = target < SHOOT_RPM;
        boolean down = target > SHOOT_RPM;
        shooter.update(getRuntime(), up, down, false, false, false, 0, 0);
    }

    /** 原地保持目标场地朝向，返回当前角度误差。 */
    private double holdAim(double targetDeg) {
        pinpoint.update();
        double headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
        setDrive(0, 0, headingControl(targetDeg, headingDeg));
        return angleErrorDeg(targetDeg, headingDeg);
    }

    /** 朝向比例控制，并给小误差保留克服静摩擦的功率。 */
    private double headingControl(double targetDeg, double currentDeg) {
        double error = angleErrorDeg(targetDeg, currentDeg);
        double power = TURN_KP * error;
        if (Math.abs(error) > 1.0 && Math.abs(power) < MIN_TURN_POWER) {
            power = Math.copySign(MIN_TURN_POWER, error);
        }
        return Range.clip(power, -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    /** 麦轮功率分配与 DriveSubsystem 一致。 */
    private void setDrive(double x, double y, double rotation) {
        double fl = y + x + rotation;
        double bl = y - x + rotation;
        double fr = y - x - rotation;
        double br = y + x - rotation;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));
        leftFrontMotor.setPower(fl / max);
        leftBackMotor.setPower(bl / max);
        rightFrontMotor.setPower(fr / max);
        rightBackMotor.setPower(br / max);
    }

    private void stopDrive() {
        setDrive(0, 0, 0);
    }

    /** 目标角度减当前角度，归一化到 [-180, 180]。 */
    private double angleErrorDeg(double targetDeg, double currentDeg) {
        double error = targetDeg - currentDeg;
        while (error > 180.0) error -= 360.0;
        while (error < -180.0) error += 360.0;
        return error;
    }
}
