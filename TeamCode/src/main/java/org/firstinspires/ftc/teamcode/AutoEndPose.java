package org.firstinspires.ftc.teamcode;

/**
 * 在同一次 Robot Controller 进程内保存自动阶段的末位置。
 * 坐标单位为英寸，航向单位为弧度；重新 INIT 不会清除这些静态字段。
 */
public final class AutoEndPose {
    private static boolean available;
    private static double xIn;
    private static double yIn;
    private static double headingRad;

    private AutoEndPose() {
    }

    public static synchronized void clear() {
        available = false;
        xIn = 0.0;
        yIn = 0.0;
        headingRad = 0.0;
    }

    public static synchronized void save(double xIn, double yIn, double headingRad) {
        AutoEndPose.xIn = xIn;
        AutoEndPose.yIn = yIn;
        AutoEndPose.headingRad = headingRad;
        available = true;
    }

    public static synchronized boolean hasValue() {
        return available;
    }

    public static synchronized double getXIn() {
        return xIn;
    }

    public static synchronized double getYIn() {
        return yIn;
    }

    public static synchronized double getHeadingRad() {
        return headingRad;
    }
}
