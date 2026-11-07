package org.firstinspires.ftc.teamcode.subsystem;

/** 双 Hive 自瞄解算：选择瞄点、计算方位角并处理微调与近距离保持。 */
public class TurretAim {

    private static final double AIM_POINT_X_IN = 60.0; // 两个 Hive 瞄点的 X 坐标
    private static final double AIM_POINT_Y_HIGH_IN = 84.0; // 高侧 Hive 的 Y 坐标
    private static final double AIM_POINT_Y_LOW_IN = 60.0; // 低侧 Hive 的 Y 坐标
    private static final double AIM_SWITCH_Y_IN = 72.0; // 高低瞄点切换基准
    private static final double AIM_SWITCH_HYST_IN = 1.0; // 切换滞回，防止抖动
    private static final double AIM_MIN_DIST_IN = 0.5; // 有效瞄准方向的最小距离

    private double aimTargetX = AIM_POINT_X_IN;
    private double aimTargetY = AIM_POINT_Y_LOW_IN;
    private double aimDistance = 0.0;
    private double aimTargetDeg = 0.0;
    private double aimOffsetDeg = 0.0;

    private boolean hasValidAim = false;
    private double lastValidAimTargetDeg = 0.0;

    /** 每圈更新瞄点和距离；机器人穿过 y=72 时带滞回切换 Hive。 */
    public void update(double robotX, double robotY) {
        if (aimTargetY == AIM_POINT_Y_LOW_IN) {
            if (robotY > AIM_SWITCH_Y_IN + AIM_SWITCH_HYST_IN) {
                aimTargetY = AIM_POINT_Y_HIGH_IN;
            }
        } else if (robotY < AIM_SWITCH_Y_IN - AIM_SWITCH_HYST_IN) {
            aimTargetY = AIM_POINT_Y_LOW_IN;
        }

        double dx = aimTargetX - robotX;
        double dy = aimTargetY - robotY;
        aimDistance = Math.hypot(dx, dy);
    }

    /**
     * 返回云台应指向的场地绝对角。
     * 太靠近瞄点时沿用最后有效方向；从未建立方向时保持当前云台朝向。
     */
    public double calculateTargetYawDeg(double currentTurretAbsDeg) {
        if (aimDistance > AIM_MIN_DIST_IN) {
            return normalizeDeg(aimTargetDeg + aimOffsetDeg);
        }

        if (hasValidAim) {
            aimTargetDeg = lastValidAimTargetDeg;
            return normalizeDeg(aimTargetDeg + aimOffsetDeg);
        }

        aimTargetDeg = normalizeDeg(currentTurretAbsDeg - aimOffsetDeg);
        return normalizeDeg(currentTurretAbsDeg);
    }

    /** 使用当前目标向量计算方位角，应在 calculateTargetYawDeg 前调用。 */
    public void updateBearing(double robotX, double robotY) {
        double dx = aimTargetX - robotX;
        double dy = aimTargetY - robotY;
        aimDistance = Math.hypot(dx, dy);

        if (aimDistance > AIM_MIN_DIST_IN) {
            aimTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
            lastValidAimTargetDeg = aimTargetDeg;
            hasValidAim = true;
        }
    }

    public void adjustOffset(double deltaDeg) {
        aimOffsetDeg = normalizeDeg(aimOffsetDeg + deltaDeg);
    }

    /** 切换自瞄时只清除旧方向，保留当前高低瞄点。 */
    public void clearDirection() {
        hasValidAim = false;
        lastValidAimTargetDeg = 0.0;
    }

    public void reset() {
        aimTargetX = AIM_POINT_X_IN;
        aimTargetY = AIM_POINT_Y_LOW_IN;
        aimDistance = 0.0;
        aimTargetDeg = 0.0;
        aimOffsetDeg = 0.0;
        clearDirection();
    }

    public double getAimTargetX() { return aimTargetX; }
    public double getAimTargetY() { return aimTargetY; }
    public double getAimDistance() { return aimDistance; }
    public double getAimTargetDeg() { return aimTargetDeg; }
    public double getAimOffsetDeg() { return aimOffsetDeg; }
    public boolean hasValidAim() { return hasValidAim; }

    private static double normalizeDeg(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
