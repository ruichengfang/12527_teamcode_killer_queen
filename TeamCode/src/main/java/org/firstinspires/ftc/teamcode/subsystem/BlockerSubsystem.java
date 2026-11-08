package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NameConstants;

/** 发射通道挡板。 */
public class BlockerSubsystem {
    private static final double CLOSED_POSITION = 0.095;
    // TODO 实机测量打开位置；原程序中 OPEN 与 CLOSED 相同，当前不会改变舵机行程。
    private static final double OPEN_POSITION = 0.095;

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
        blockerServo.setPosition(open ? OPEN_POSITION : CLOSED_POSITION);
    }

    public boolean isOpen() {
        return open;
    }

    public double getPosition() {
        return open ? OPEN_POSITION : CLOSED_POSITION;
    }
}
