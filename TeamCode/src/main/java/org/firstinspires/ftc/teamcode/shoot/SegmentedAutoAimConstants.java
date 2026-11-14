package org.firstinspires.ftc.teamcode.shoot;

import com.acmerobotics.dashboard.config.Config;

/** Dashboard 配置。距离以英寸计，舵机位置为 0–1，飞轮转速为 RPM。 */
@Config
public final class SegmentedAutoAimConstants {
    // 中央禁射区独立于射击参数表。
    public static double centerHalfWidthIn = 18.0;

    // 每行依次是到 Hive 瞄点的距离（in）、飞轮转速（RPM）、pitchServo 位置。
    public static double[][] shotTable = {
            {32.0, 3305.0, 0.49},
            {37.2, 3315.0, 0.50},
            {42.6, 3365.0, 0.50},
            {47.3, 3375.0, 0.53},
            {51.3, 3475.0, 0.54},
            {62.5, 3575.0, 0.53},
    };

    private SegmentedAutoAimConstants() {}
}
/*37.2 3315 0.5
*32 3305 0.49
*42.6 3365 0.5
*47.3 3375 0.53
*51.3 3475 0.54
*62.5 3575 0.53
*
*
*
* */
