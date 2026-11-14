package org.firstinspires.ftc.teamcode.shoot;

import com.acmerobotics.dashboard.config.Config;

/** Dashboard 配置。距离以英寸计，舵机位置为 0–1，飞轮转速为 RPM。 */
@Config
public final class SegmentedAutoAimConstants {
    // 中央禁射区独立于射击参数表。
    public static double centerHalfWidthIn = 18.0;

    // 每行依次是距离下限、距离上限（in）、飞轮转速（RPM）、pitchServo 位置。
    // 区间下限不包含（0 除外）、上限包含。上限填 -1 表示无上限。
    public static double[][] shotTable = {
            {29, 35, 3185.714285714286, 0.51},//边界29~35-3185-0.51
            {35, -1, 3285.714285714286, 0.5},//35~-3285-0.5
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