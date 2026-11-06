package org.firstinspires.ftc.teamcode;

public class AutoAim {

    // Kp / Kd 可在运行中动态调整（MecanumDrive 按键调参）
    private double kp = 0.01;
    private double kd = 0.0;

    private double lastError = 0;
    private long lastTimeNs = 0;

    /**
     * 计算转向输出值
     * @param targetDegree         目标角度（度），0 表示场地方向
     * @param currentHeadingRadians 当前车头朝向（弧度）
     * @return 返回 rx 修正量，直接加到摇杆 rx 上即可
     */
    public double update(double targetDegree, double currentHeadingRadians) {
        long nowNs = System.nanoTime();
        boolean firstCall = (lastTimeNs == 0);
        double loopTime = (nowNs - lastTimeNs) / 1_000_000_000.0;
        if (loopTime < 0.001 || firstCall) loopTime = 0.001;
        lastTimeNs = nowNs;

        double error = normalizeAngle(targetDegree - Math.toDegrees(currentHeadingRadians));
        // 首帧不做微分，避免刚按下按键时猛打方向
        double derivative = firstCall ? 0 : (error - lastError) / loopTime;
        double output =(kp * error + kd * derivative);

        lastError = error;
        return output;
    }

    /** 动态调参：不限范围，正负由调用方决定 */
    public void adjustKp(double delta) { kp += delta; }
    public void adjustKd(double delta) { kd += delta; }
    public double getKp() { return kp; }
    public double getKd() { return kd; }

    public void reset() {
        lastError = 0;
        lastTimeNs = 0;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180)   angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}