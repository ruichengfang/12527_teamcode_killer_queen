package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.NameConstants;

/** 副操俯仰舵机位置控制。 */
@Config
public final class PitchSubsystem extends SubsystemBase {
    public static double pitchInitialPosition = 0.52;
    public static double pitchStep = 0.01;
    private final Servo pitchServo;
    private double position;

    public PitchSubsystem(HardwareMap hardwareMap) {
        pitchServo = hardwareMap.tryGet(Servo.class, NameConstants.PITCH_SERVO);
        position = Double.isFinite(pitchInitialPosition) ? Math.max(0, Math.min(1, pitchInitialPosition)) : 0.52;
        if (pitchServo != null) pitchServo.setPosition(position);
    }

    public void adjust(int direction) {
        if (!Double.isFinite(pitchStep) || pitchStep < 0) return;
        setPosition(position + direction * pitchStep);
    }

    public void setPosition(double requestedPosition) {
        if (pitchServo == null || !Double.isFinite(requestedPosition)) return;
        double next = Math.max(0, Math.min(1, requestedPosition));
        if (next == position) return;
        position = next;
        pitchServo.setPosition(position);
    }
    public boolean isAvailable() { return pitchServo != null; }
    public double getPosition() { return position; }
}
