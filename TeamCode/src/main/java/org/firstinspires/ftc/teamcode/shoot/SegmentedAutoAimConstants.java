package org.firstinspires.ftc.teamcode.shoot;

import com.acmerobotics.dashboard.config.Config;

/** Dashboard 配置。距离以英寸计，舵机位置为 0–1，飞轮转速为 RPM。 */
@Config
public final class SegmentedAutoAimConstants {
    // 中央禁射区独立于射击参数表。
    public static double centerHalfWidthIn = 18.0;

    // 每行依次是距 Hive 瞄点的水平距离（in）、飞轮转速（RPM）、pitchServo 位置。
    public static double[][] shotTable = {
            {20.0, 3385.714285714286, 0.52},
            {32.0, 3385.714285714286, 0.52},
            {43.0, 3385.714285714286, 0.52},
    };

    private SegmentedAutoAimConstants() {}
}
