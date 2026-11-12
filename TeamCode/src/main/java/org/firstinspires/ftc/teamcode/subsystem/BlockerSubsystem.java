package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.NameConstants;

/** 发射通道挡板（配置名 blockerServo）。 */
@Config
public class BlockerSubsystem extends SubsystemBase {
    // 位置历史：0.095/0.095 → 0.11/0.10 → 0.38/0.55 → 11-11 21:43 队友改为 0.3/0.6。
    // 2026-11-12 实车反馈“一直是开的”：说明 0.3 实际不是挡住的位置。在 Dashboard
    // Config → BlockerSubsystem 里实时改 CLOSED_POSITION（松开 RT 时生效），找到挡住的值后写回这里。
    // 2026-11-12 用户实车确认：0.0 关闭（挡住），0.3 打开。
    public static double CLOSED_POSITION = 0.0;
    public static double OPEN_POSITION = 0.3;

    private double testposition = CLOSED_POSITION;

    private final Servo blockerServo;
    private boolean open;

    public BlockerSubsystem(HardwareMap hardwareMap) {
        blockerServo = hardwareMap.get(Servo.class, NameConstants.BLOCKER);
        blockerServo.setPosition(CLOSED_POSITION);
    }

    public void toggle() {
        setOpen(!open);
    }

    public void setOpen(boolean open) {
        this.open = open;
        testposition = open ? OPEN_POSITION : CLOSED_POSITION;
        blockerServo.setPosition(testposition);
    }

    public boolean isOpen() {
        return open;
    }

    public double getPosition() {
        return testposition;
    }

    public void addposition() {
        testposition += 0.01;
    }

    public void minposition() {
        testposition -= 0.01;
    }

    /** 在控制命令完成按键处理后写出，避免调度器 periodic 的前置顺序改变输出。 */
    public void flushOutput() {
        testposition = open ? OPEN_POSITION : CLOSED_POSITION;
        blockerServo.setPosition(testposition);
    }
}
