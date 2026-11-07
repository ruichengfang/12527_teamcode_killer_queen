package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NameConstants;

/** Blocker 舵机：初始关闭，左 Bumper 上升沿切换开关。 */
public class Blocker {

    private static final double CLOSED_POS = 0.11; // 关闭位置
    private static final double OPEN_POS = 0.10; // 打开测试位置，与原 0.10 拉开动作幅度

    private final Servo blockerServo;
    private boolean blockerOpen = false;
    private boolean prevLeftBumper = false;

    public Blocker(HardwareMap hardwareMap) {
        blockerServo = hardwareMap.get(Servo.class, NameConstants.BLOCKER_SERVO);
        close();
    }

    /** 在 waitForStart() 后调用，清除按键边沿状态。 */
    public void start() {
        prevLeftBumper = false;
    }

    /** 每圈调用一次，左 Bumper 每按下一次切换一次状态。 */
    public void update(Gamepad gamepad, Telemetry telemetry) {
        boolean leftBumper = gamepad.left_bumper;
        if (leftBumper && !prevLeftBumper) {
            blockerOpen = !blockerOpen;
        }
        prevLeftBumper = leftBumper;
        // 每帧发送当前目标，避免初始化或其他状态导致命令未刷新。
        applyPosition();

        telemetry.addData("LB 输入", leftBumper);
        telemetry.addData("Blocker 状态", blockerOpen ? "打开 (LB 关闭)" : "关闭 (LB 打开)");
        telemetry.addData("Blocker 舵机位置", "%.2f",
                blockerOpen ? OPEN_POS : CLOSED_POS);
    }

    public void open() {
        blockerOpen = true;
        applyPosition();
    }

    public void close() {
        blockerOpen = false;
        applyPosition();
    }

    public boolean isOpen() {
        return blockerOpen;
    }

    private void applyPosition() {
        blockerServo.setPosition(blockerOpen ? OPEN_POS : CLOSED_POS);
    }
}
