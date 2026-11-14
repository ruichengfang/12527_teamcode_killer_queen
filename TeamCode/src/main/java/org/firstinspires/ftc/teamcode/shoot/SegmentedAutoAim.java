package org.firstinspires.ftc.teamcode.shoot;

import java.util.Locale;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 从射击表中选出包含当前距离的区间。 */
public final class SegmentedAutoAim {
    private static final int MIN_DISTANCE = 0;
    private static final int MAX_DISTANCE = 1;
    public static final int RPM = 2;
    public static final int PITCH = 3;
    private static final double UNBOUNDED = -1.0;

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
        double[][] table = SegmentedAutoAimConstants.shotTable;
        if (!Double.isFinite(distanceIn) || distanceIn < 0.0 || table == null) return null;

        for (double[] row : table) {
            if (row == null || row.length != 4
                    || !Double.isFinite(row[RPM]) || row[RPM] <= 0.0
                    || !Double.isFinite(row[PITCH]) || row[PITCH] < 0.0 || row[PITCH] > 1.0) continue;
            double min = row[MIN_DISTANCE];
            double max = row[MAX_DISTANCE];
            if (!Double.isFinite(min) || min < 0.0 || !Double.isFinite(max)) continue;
            if ((min == 0.0 || distanceIn > min) && (max == UNBOUNDED || distanceIn <= max)) return row;
        }
        return null;
    }

    public static String getProfileLabel(double[] profile) {
        if (profile == null) return "无可用射击档";
        double min = profile[MIN_DISTANCE];
        double max = profile[MAX_DISTANCE];
        if (max == UNBOUNDED) return String.format(Locale.US, ">%.1f in", min);
        return String.format(Locale.US, "%s%.1f, %.1f] in", min == 0.0 ? "[" : "(", min, max);
    }
}
