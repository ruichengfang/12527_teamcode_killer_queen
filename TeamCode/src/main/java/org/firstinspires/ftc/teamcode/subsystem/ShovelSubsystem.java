package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.NameConstants;

/** 铲子舵机：Y 键在上下位置之间切换。 */
@Config
public final class ShovelSubsystem extends SubsystemBase {
    // 首次切换前不发送舵机位置。
    public static double shovelUpPosition = 0.55;
    public static double shovelDownPosition = 0.15;

    private final Servo flowerServo;
    private boolean hasTarget;
    private boolean shovelUp;

    public ShovelSubsystem(HardwareMap hardwareMap) {
        flowerServo = hardwareMap.tryGet(Servo.class, NameConstants.flowerServo);
    }

    public void toggle() {
        if (flowerServo == null) return;
        shovelUp = !shovelUp;
        hasTarget = true;
        writeTarget();
    }

    @Override public void periodic() {
        if (hasTarget) writeTarget();
    }

    private void writeTarget() {
        double target = shovelUp ? shovelUpPosition : shovelDownPosition;
        if (Double.isFinite(target)) flowerServo.setPosition(Math.max(0.0, Math.min(1.0, target)));
    }

    public boolean isAvailable() { return flowerServo != null; }
    public boolean hasTarget() { return hasTarget; }
    public boolean isShovelUp() { return shovelUp; }
}
