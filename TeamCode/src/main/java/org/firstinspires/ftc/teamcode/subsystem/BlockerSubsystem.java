package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.NameConstants;

/** 发射通道挡板。 */
public class BlockerSubsystem extends SubsystemBase {
    // 恢复此前程序中的位置，需根据实物确认行程是否足够。
    private static final double CLOSED_POSITION = 0.38;
    private static final double OPEN_POSITION = 0.55;

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
    public void addposition(){
        testposition += 0.01;
    }

    public void minposition(){
        testposition -= 0.01;
    }
    @Override
    public void periodic(){
        blockerServo.setPosition(testposition);
    }
}
