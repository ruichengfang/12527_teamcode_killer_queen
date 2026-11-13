package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.NameConstants;

/** 副操使用 LB / RB 调试 pitchServo。 */
@TeleOp(name = "Pitch Servo Calibration", group = "Test")
public final class PitchServoCalibration extends OpMode {
    private static final double START_POSITION = 0.50;
    private static final double POSITION_STEP = 0.01;
    private Servo pitchServo;
    private double position;
    private boolean previousUp, previousDown;

    @Override public void init() {
        pitchServo = hardwareMap.get(Servo.class, NameConstants.PITCH_SERVO);
        position = START_POSITION;
        telemetry.addLine("按 START 将 pitchServo 设到 0.50；副操 LB / RB 每次调整 0.01");
        telemetry.update();
    }

    @Override public void start() {
        previousUp = gamepad2.right_bumper;
        previousDown = gamepad2.left_bumper;
        pitchServo.setPosition(position);
    }

    @Override public void loop() {
        boolean up = gamepad2.right_bumper, down = gamepad2.left_bumper;
        if (up && !previousUp && !down) position = Range.clip(position + POSITION_STEP, 0.0, 1.0);
        else if (down && !previousDown && !up) position = Range.clip(position - POSITION_STEP, 0.0, 1.0);
        if ((up && !previousUp && !down) || (down && !previousDown && !up)) pitchServo.setPosition(position);
        previousUp = up;
        previousDown = down;
        telemetry.addData("pitchServo 指令位置", "%.2f", position);
        telemetry.update();
    }
}
