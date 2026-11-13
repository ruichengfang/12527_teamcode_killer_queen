package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.NameConstants;

/** 铲子舵机：Y 键在上下位置之间切换。 */
@Config
public final class ShovelSubsystem extends SubsystemBase {
    // 占位参数，实车测量后修改；两个位置相同，首次切换前不发送舵机位置。
    public static double shovelUpPosition = 0.5;
    public static double shovelDownPosition = 0.5;

    private final Servo shovelServo;
    private boolean hasTarget;
    private boolean shovelUp;

    public ShovelSubsystem(HardwareMap hardwareMap) {
        shovelServo = hardwareMap.tryGet(Servo.class, NameConstants.shovelServo);
    }

    public void toggle() {
        if (shovelServo == null) return;
        shovelUp = !shovelUp;
        hasTarget = true;
        writeTarget();
    }

    @Override public void periodic() {
        if (hasTarget) writeTarget();
    }

    private void writeTarget() {
        double target = shovelUp ? shovelUpPosition : shovelDownPosition;
        if (Double.isFinite(target)) shovelServo.setPosition(Math.max(0.0, Math.min(1.0, target)));
    }

    public boolean isAvailable() { return shovelServo != null; }
    public boolean hasTarget() { return hasTarget; }
    public boolean isShovelUp() { return shovelUp; }
}
