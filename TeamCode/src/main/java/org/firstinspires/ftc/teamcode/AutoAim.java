//package org.firstinspires.ftc.teamcode;
//
//import org.firstinspires.ftc.teamcode.subSystems.servo;
//
//public class AutoAim {
//    private static final double Kp = 0.001;
//    private static final double Kd = 0.001;
//    private static final double OUTPUT_SCALE = 0.4;
//
//    private double lastError = 0;
//    private long lastTimeNs = 0;
//
//    /**
//     * 计算转向输出值
//     * @param targetDegree         目标角度（度），0 表示场地方向
//     * @param currentHeadingRadians 当前车头朝向（弧度）
//     * @return 返回 rx 修正量，直接加到摇杆 rx 上即可
//     */
//    public double update(double targetDegree, double currentHeadingRadians, servo Servo) {
//        long nowNs = System.nanoTime();
//        double loopTime = (nowNs - lastTimeNs) / 1_000_000_000.0;
//        if (loopTime < 0.001 || lastTimeNs == 0) loopTime = 0.001;
//        lastTimeNs = nowNs;
//
//        double error = normalizeAngle(targetDegree - Math.toDegrees(currentHeadingRadians));
//        double derivative = (error - lastError) / loopTime;
//        double output = OUTPUT_SCALE * (Kp * error + Kd * derivative);
//
//        Servo.update(false, false, true);
//
//        lastError = error;
//        return output;
//    }
//
//    public void reset() {
//        lastError = 0;
//        lastTimeNs = 0;
//    }
//
//    private double normalizeAngle(double angle) {
//        while (angle > 180)   angle -= 360;
//        while (angle <= -180) angle += 360;
//        return angle;
//    }
//}