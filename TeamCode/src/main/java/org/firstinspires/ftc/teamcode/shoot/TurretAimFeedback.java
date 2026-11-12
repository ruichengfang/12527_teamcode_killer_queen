package org.firstinspires.ftc.teamcode.shoot;

/** 根据目标有效性和舵机指令估算瞄准状态；没有舵机位置反馈。 */
public final class TurretAimFeedback {
    public static final double SETTLE_SECONDS = 0.2;
    public static final double STABLE_ANGLE_DEG = 3.0;
    private boolean settling, aimed;
    private double since, referenceAngle;

    /** 从未瞄准进入估计瞄准状态时返回 true，供调用者触发一次震动。 */
    public boolean update(boolean valid, double angle, double now) {
        if (!valid || !Double.isFinite(angle) || !Double.isFinite(now)) {
            reset();
            return false;
        }
        if (!settling || Math.abs(angle - referenceAngle) > STABLE_ANGLE_DEG || now < since) {
            referenceAngle = angle;
            since = now;
            settling = true;
            aimed = false;
        }
        if (!aimed && now - since >= SETTLE_SECONDS) {
            aimed = true;
            return true;
        }
        return false;
    }

    public boolean isAimed() { return aimed; }

    public void reset() { settling = aimed = false; }
}
