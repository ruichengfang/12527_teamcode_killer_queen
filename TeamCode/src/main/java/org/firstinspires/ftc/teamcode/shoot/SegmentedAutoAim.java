package org.firstinspires.ftc.teamcode.shoot;

import java.util.Locale;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 从射击表中选出距离最接近的一行。 */
public final class SegmentedAutoAim {
    public static final int DISTANCE = 0;
    public static final int RPM = 1;
    public static final int PITCH = 2;

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
        return findClosestProfile(turret.getAimDistance());
    }

    public static double[] findClosestProfile(double distanceIn) {
        double[][] table = SegmentedAutoAimConstants.shotTable;
        if (!Double.isFinite(distanceIn) || distanceIn < 0.0 || table == null) return null;

        double[] closest = null;
        double smallestError = Double.POSITIVE_INFINITY;
        for (double[] row : table) {
            if (row == null || row.length != 3
                    || !Double.isFinite(row[DISTANCE]) || row[DISTANCE] < 0.0
                    || !Double.isFinite(row[RPM]) || row[RPM] <= 0.0
                    || !Double.isFinite(row[PITCH]) || row[PITCH] < 0.0 || row[PITCH] > 1.0) continue;
            double error = Math.abs(distanceIn - row[DISTANCE]);
            // 距离差相同时保留表中靠前的一行。
            if (error < smallestError) {
                smallestError = error;
                closest = row;
            }
        }
        return closest;
    }

    public static String getProfileLabel(double[] profile) {
        return profile == null ? "无可用射击档" : String.format(Locale.US, "%.1f in 档", profile[DISTANCE]);
    }
}
