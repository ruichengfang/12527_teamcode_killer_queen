//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.subSystems.inTake;
//import org.firstinspires.ftc.teamcode.subSystems.shooter;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
///**
// * AutoPath（无 Pedro Pathing 版本）
// *
// * 流程与 visualizer 导出的路径一致：
// *   1. 从 start 出发，闭环移动到 path1 (58.4825, 109.6616, 270)；
// *   2. 原地转向最近的红方 hive，飞轮加速后供弹射击；
// *   3. 继续移动到 point2 (15.3086, 110.0113, 179.5359)。
// *
// * 定位：goBILDA Pinpoint（设备名 "od"，配置与 MecanumDrive 完全一致）。
// * 坐标系：x/y 单位英寸（0~144），heading 单位度，逆时针为正，0° 朝 +x 方向。
// */
//@Autonomous(name = "AutoPath", group = "Autonomous")
//public class AutoPath extends LinearOpMode {
//
//    /* ==================== 路径点（原 AutoPath 的三个坐标） ==================== */
//    private static final double[] START  = {58.5332, 133.3355, 270};
//    private static final double[] PATH1  = {58.4825, 109.6616, 270};
//    private static final double[] POINT2 = {15.3086, 110.0113, 179.5359};
//
//    /* ==================== 红方 hive 坐标 ==================== */
//    // 红方 hive（球门）中心坐标，单位英寸。
//    // 有多个红方目标可继续添加，程序自动选离当前位置最近的一个。
//    private static final double[][] RED_HIVES = {
//            {58.75, 94.75}
//    };
//
//    /* ==================== 射击参数 ==================== */
//    private static final double SHOOT_RPM = 3500;        // 【需修改】飞轮目标转速
//    private static final double RPM_READY_RATIO = 0.95;  // 实际转速达到目标 95% 即供弹
//    private static final double SPINUP_TIMEOUT_S = 4.0;  // 转速等待超时保护
//    private static final double FEED_TIME_S = 3.0;       // 【需修改】供弹持续时间
//    private static final double AIM_TOLERANCE_DEG = 2.0; // 对准角度容差
//    private static final double AIM_TIMEOUT_S = 3.0;     // 对准超时保护
//
//    /* ==================== 走点控制参数 ==================== */
//    private static final double DRIVE_KP = 0.08;         // 每英寸误差对应的功率
//    private static final double MAX_DRIVE_POWER = 0.7;   // 平移最大功率
//    private static final double MIN_DRIVE_POWER = 0.15;  // 克服静摩擦的最小功率
//    private static final double MAX_TURN_POWER = 0.6;    // 转向最大功率
//    private static final double X2_FACTOR = 1.1;         // 与 MecanumDrive 的 x2Factor 一致
//    private static final double POS_TOLERANCE = 1.0;     // 到位容差（英寸）
//    private static final double HEAD_TOLERANCE_DEG = 3.0;
//    private static final double GO_TO_TIMEOUT_S = 8.0;   // 每段走点超时保护
//
//    /* ==================== 硬件 ==================== */
//    private DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
//    private GoBildaPinpointDriver pinpoint;
//    private AutoAim autoAim;
//    private shooter shooter;
//    private inTake intake;
//
//    @Override
//    public void runOpMode() {
//        // ---------- 底盘电机（名称/方向与 MecanumDrive 一致） ----------
//        leftFrontMotor  = hardwareMap.get(DcMotor.class, NameConstants.LEFT_FRONT_MOTOR);
//        rightFrontMotor = hardwareMap.get(DcMotor.class, NameConstants.RIGHT_FRONT_MOTOR);
//        leftBackMotor   = hardwareMap.get(DcMotor.class, NameConstants.LEFT_BACK_MOTOR);
//        rightBackMotor  = hardwareMap.get(DcMotor.class, NameConstants.RIGHT_BACK_MOTOR);
//        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
//        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        // ---------- Pinpoint 定位（配置与 MecanumDrive 一致） ----------
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, NameConstants.OD);
//        pinpoint.setEncoderDirections(
//                GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        pinpoint.setOffsets(91.0, 78.0, DistanceUnit.MM);
//        pinpoint.resetPosAndIMU();
//        pinpoint.update();
//        // 把起点设为 visualizer 中的 start 坐标
//        pinpoint.setPosition(new Pose2D(
//                DistanceUnit.INCH, START[0], START[1], AngleUnit.DEGREES, START[2]));
//
//        // ---------- 子系统 ----------
//        autoAim = new AutoAim();
//        shooter = new shooter(hardwareMap);
//        intake = new inTake(hardwareMap);
//
//        telemetry.addLine("Ready.");
//        telemetry.update();
//
//        waitForStart();
//
//        // 1. 移动到射击点
//        goTo(PATH1);
//        // 2. 对准最近的红方 hive 并射击
//        shootAtHive();
//        // 3. 继续原路径第二段
//        goTo(POINT2);
//
//        stopDrive();
//    }
//
//    /* ==================== 走点 ==================== */
//
//    /** 闭环移动到目标点（x, y, heading），带超时保护。 */
//    private void goTo(double[] target) {
//        autoAim.reset();
//        double t0 = getRuntime();
//
//        while (opModeIsActive()) {
//            pinpoint.update();
//            Pose2D pos = pinpoint.getPosition();
//            double x = pos.getX(DistanceUnit.INCH);
//            double y = pos.getY(DistanceUnit.INCH);
//            double heading = pos.getHeading(AngleUnit.RADIANS);
//
//            double dx = target[0] - x;
//            double dy = target[1] - y;
//            double dist = Math.hypot(dx, dy);
//            double headErr = angleErrorDeg(target[2], Math.toDegrees(heading));
//
//            if (dist < POS_TOLERANCE && Math.abs(headErr) < HEAD_TOLERANCE_DEG) break;
//            if (getRuntime() - t0 > GO_TO_TIMEOUT_S) break;
//
//            // 场地方向的目标向量 → 机器人坐标系（与 MecanumDrive 相同的变换）
//            double dir = Math.atan2(dy, dx) - heading;
//            double size = Math.min(MAX_DRIVE_POWER, dist * DRIVE_KP);
//            if (dist > POS_TOLERANCE && size < MIN_DRIVE_POWER) size = MIN_DRIVE_POWER;
//
//            double x2 = size * Math.cos(dir) * X2_FACTOR;
//            double y2 = size * Math.sin(dir);
//            double rx = Range.clip(autoAim.update(target[2], heading,),
//                    -MAX_TURN_POWER, MAX_TURN_POWER);
//
//            setDrive(x2, y2, rx);
//
//            telemetry.addData("阶段", "goTo");
//            telemetry.addData("x", "%.1f", x);
//            telemetry.addData("y", "%.1f", y);
//            telemetry.addData("heading deg", "%.1f", Math.toDegrees(heading));
//            telemetry.addData("距离目标", "%.1f in", dist);
//            telemetry.update();
//        }
//        stopDrive();
//    }
//
//    /* ==================== 对准射击 ==================== */
//
//    private void shootAtHive() {
//        // 用实测位置计算最近 hive 的方位角
//        pinpoint.update();
//        Pose2D pos = pinpoint.getPosition();
//        double x = pos.getX(DistanceUnit.INCH);
//        double y = pos.getY(DistanceUnit.INCH);
//        double[] hive = nearestRedHive(x, y);
//        double targetDeg = Math.toDegrees(Math.atan2(hive[1] - y, hive[0] - x));
//
//        // 1. 原地转向 hive，同时飞轮开始加速
//        autoAim.reset();
//        double t0 = getRuntime();
//        while (opModeIsActive()) {
//            spinUpTick();
//            double err = holdAim(targetDeg);
//            if (Math.abs(err) < AIM_TOLERANCE_DEG) break;
//            if (getRuntime() - t0 > AIM_TIMEOUT_S) break;
//        }
//
//        // 2. 保持对准，等飞轮转速就位
//        t0 = getRuntime();
//        while (opModeIsActive()
//                && shooter.getFilteredRPM() < SHOOT_RPM * RPM_READY_RATIO
//                && getRuntime() - t0 < SPINUP_TIMEOUT_S) {
//            spinUpTick();
//            holdAim(targetDeg);
//        }
//
//        // 3. 供弹射击：电机 2/3 正转推弹 + intake 送球，全程保持对准
//        t0 = getRuntime();
//        while (opModeIsActive() && getRuntime() - t0 < FEED_TIME_S) {
//            shooter.update(getRuntime(), false, false, false, true, false, 0, 0);
//            intake.intake();
//            holdAim(targetDeg);
//
//            telemetry.addData("阶段", "shoot");
//            telemetry.addData("目标角度", "%.1f", targetDeg);
//            telemetry.addData("飞轮RPM", "%.0f", shooter.getFilteredRPM());
//            telemetry.update();
//        }
//
//        // 4. 停射：飞轮目标清零，intake 回到低速常态
//        shooter.update(getRuntime(), false, false, true, false, false, 0, 0);
//        intake.constant();
//        stopDrive();
//    }
//
//    /** 找到离 (x, y) 最近的红方 hive，返回 {x, y}。 */
//    private double[] nearestRedHive(double x, double y) {
//        double[] best = RED_HIVES[0];
//        double bestDist = Double.MAX_VALUE;
//        for (double[] hive : RED_HIVES) {
//            double dx = hive[0] - x;
//            double dy = hive[1] - y;
//            double dist = dx * dx + dy * dy;
//            if (dist < bestDist) {
//                bestDist = dist;
//                best = hive;
//            }
//        }
//        return best;
//    }
//
//    /** 每周期调用：模拟十字键上/下，把飞轮目标转速稳定在 SHOOT_RPM。 */
//    private void spinUpTick() {
//        double target = shooter.getTargetRPM();
//        boolean up = target < SHOOT_RPM;
//        boolean down = target > SHOOT_RPM;
//        shooter.update(getRuntime(), up, down, false, false, false, 0, 0);
//    }
//
//    /** 原地保持/修正朝向，返回当前角度误差（度）。 */
//    private double holdAim(double targetDeg) {
//        pinpoint.update();
//        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
//        double rx = Range.clip(autoAim.update(targetDeg, heading),
//                -MAX_TURN_POWER, MAX_TURN_POWER);
//        setDrive(0, 0, rx);
//        return angleErrorDeg(targetDeg, Math.toDegrees(heading));
//    }
//
//    /* ==================== 底盘与工具 ==================== */
//
//    /** 功率分配与 MecanumDrive 完全一致。 */
//    private void setDrive(double x2, double y2, double rx) {
//        double fl = y2 + x2 + rx;
//        double bl = y2 - x2 + rx;
//        double fr = y2 - x2 - rx;
//        double br = y2 + x2 - rx;
//
//        double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl),
//                Math.max(Math.abs(fr), Math.abs(br))));
//        if (max > 1.0) {
//            fl /= max;
//            bl /= max;
//            fr /= max;
//            br /= max;
//        }
//
//        leftFrontMotor.setPower(fl);
//        leftBackMotor.setPower(bl);
//        rightFrontMotor.setPower(fr);
//        rightBackMotor.setPower(br);
//    }
//
//    private void stopDrive() {
//        setDrive(0, 0, 0);
//    }
//
//    /** 目标角度 - 当前角度，归一化到 [-180, 180]。 */
//    private double angleErrorDeg(double targetDeg, double currentDeg) {
//        double err = targetDeg - currentDeg;
//        while (err > 180)  err -= 360;
//        while (err < -180) err += 360;
//        return err;
//    }
//}
