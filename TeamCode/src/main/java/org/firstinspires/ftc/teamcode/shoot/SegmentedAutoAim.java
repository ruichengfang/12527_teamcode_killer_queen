package org.firstinspires.ftc.teamcode.shoot;

import java.util.Locale;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 在相邻实测点之间分别线性插值飞轮转速和 pitch。 */
public final class SegmentedAutoAim {
    private static final int DISTANCE = 0;
    public static final int RPM = 1;
    public static final int PITCH = 2;
    private static final double PITCH_CHANGE_TO_REOPEN = 0.01;

    private SegmentedAutoAim() {}

    public static boolean isInCenter(LocalizationSubsystem localization) {
        return localization.isFieldPoseValid()
                && Math.abs(localization.getXIn() - LocalizationSubsystem.OPTION_RESET_X_IN)
                    <= SegmentedAutoAimConstants.centerHalfWidthIn
                && Math.abs(localization.getYIn() - LocalizationSubsystem.OPTION_RESET_Y_IN)
                    <= SegmentedAutoAimConstants.centerHalfWidthIn;
    }

    public static double[] select(LocalizationSubsystem localization, TurretSubsystem turret) {
        if (!localization.isFieldPoseValid() || !turret.hasValidAim()) return null;
        return findProfileForDistance(turret.getAimDistance());
    }

    public static double[] findProfileForDistance(double distanceIn) {
        double[][] points = findSurroundingPoints(distanceIn);
        if (points == null) return null;
        double[] lower = points[0];
        double[] upper = points[1];
        if (lower == upper) return new double[]{distanceIn, lower[RPM], lower[PITCH]};

        double fraction = (distanceIn - lower[DISTANCE]) / (upper[DISTANCE] - lower[DISTANCE]);
        return new double[]{distanceIn,
                lower[RPM] + fraction * (upper[RPM] - lower[RPM]),
                lower[PITCH] + fraction * (upper[PITCH] - lower[PITCH])};
    }

    /** 测试手动编辑最近的实测行，实际射击再按当前距离重新插值。 */
    public static double[] findNearestCalibrationPoint(double distanceIn) {
        double[][] points = findSurroundingPoints(distanceIn);
        if (points == null) return null;
        double[] lower = points[0];
        double[] upper = points[1];
        return distanceIn - lower[DISTANCE] <= upper[DISTANCE] - distanceIn ? lower : upper;
    }

    private static double[][] findSurroundingPoints(double distanceIn) {
        double[][] table = SegmentedAutoAimConstants.shotTable;
        if (!Double.isFinite(distanceIn) || distanceIn < 0.0 || table == null) return null;

        double[] lower = null;
        double[] upper = null;
        for (double[] row : table) {
            if (row == null || row.length != 3
                    || !Double.isFinite(row[DISTANCE]) || row[DISTANCE] < 0.0
                    || !Double.isFinite(row[RPM]) || row[RPM] <= 0.0
                    || !Double.isFinite(row[PITCH]) || row[PITCH] < 0.0 || row[PITCH] > 1.0) continue;
            double sampleDistance = row[DISTANCE];
            if (sampleDistance <= distanceIn && (lower == null || sampleDistance > lower[DISTANCE])) lower = row;
            if (sampleDistance >= distanceIn && (upper == null || sampleDistance < upper[DISTANCE])) upper = row;
        }
        if (lower == null && upper == null) return null;
        return new double[][]{lower == null ? upper : lower, upper == null ? lower : upper};
    }

    /** 以开挡板时的参数为基准，过滤距离测量的小幅波动。 */
    public static boolean hasFeedTargetChanged(double rpm, double pitch, double openedRpm, double openedPitch) {
        return Math.abs(rpm - openedRpm) > ShooterSubsystem.TARGET_CHANGE_RESET_RPM
                || Math.abs(pitch - openedPitch) > PITCH_CHANGE_TO_REOPEN;
    }

    public static String getProfileLabel(double[] profile) {
        return profile == null ? "无可用射击参数" : String.format(Locale.US, "%.1f in", profile[DISTANCE]);
    }
}
