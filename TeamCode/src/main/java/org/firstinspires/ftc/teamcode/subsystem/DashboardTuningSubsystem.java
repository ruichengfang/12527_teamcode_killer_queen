package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.tests.DashTuner.*;

import android.graphics.Color;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** DashTuner 独立诊断硬件；配置入口仍是原来的 DashTuner。 */
public final class DashboardTuningSubsystem extends SubsystemBase {
    private final DcMotorEx[] motors = new DcMotorEx[4];

    private final Servo[] servos = new Servo[4];

    private final PIDFController[] pidfControllers = {
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0)
    };

    private final ColorSensor[] colorSensor = new ColorSensor[4];

    private final DistanceSensor[] distanceSensor = new DistanceSensor[4];

    private final float[] hsvValues = {0F, 0F, 0F};

    private static final double SCALE_FACTOR = 255;

    private final FtcDashboard dashboard;

    public DashboardTuningSubsystem(HardwareMap hardwareMap) {
        dashboard = FtcDashboard.getInstance();
        for (int i = 0; i < 4; i++) {
            if (!motorName[i].isEmpty()) {
                motors[i] = hardwareMap.get(DcMotorEx.class, motorName[i]);
                if (isVelocityCloseLoop[i]) {
                    motors[i].setPIDFCoefficients(
                            DcMotor.RunMode.RUN_USING_ENCODER,
                            new PIDFCoefficients(
                                    kP[i],
                                    kI[i],
                                    kD[i],
                                    kF[i]
                            )
                    );
                }
                if (isPositionCloseLoop[i]) {
                    pidfControllers[i].setPIDF(kP[i], kI[i], kD[i], kF[i]);
                }
            }
            if (!servoName[i].isEmpty()) {
                servos[i] = hardwareMap.get(Servo.class, servoName[i]);
            }
            if (!colorSensorName[i].isEmpty()) {
                colorSensor[i] = hardwareMap.get(ColorSensor.class, colorSensorName[i]);
                distanceSensor[i] = hardwareMap.get(DistanceSensor.class, colorSensorName[i]);
            }
        }
    }

    public void updateOutputs() {
        for (int i = 0; i < 4; i++) {
            if (!motorName[i].isEmpty()) {
                if (slaveTo[i] != -1) {
                    if (isVelocityCloseLoop[slaveTo[i]] && !isPositionCloseLoop[slaveTo[i]]) {
                        motors[i].setPIDFCoefficients(
                                DcMotor.RunMode.RUN_USING_ENCODER,
                                new PIDFCoefficients(
                                        kP[slaveTo[i]],
                                        kI[slaveTo[i]],
                                        kD[slaveTo[i]],
                                        kF[slaveTo[i]]
                                )
                        );
                        motors[i].setVelocity(-motorTarget[slaveTo[i]]);
                    }
                } else if (isVelocityCloseLoop[i] && !isPositionCloseLoop[i]) {
                    motors[i].setPIDFCoefficients(
                            DcMotor.RunMode.RUN_USING_ENCODER,
                            new PIDFCoefficients(
                                    kP[i],
                                    kI[i],
                                    kD[i],
                                    kF[i]
                            )
                    );

                    motors[i].setVelocity(motorTarget[i]);

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("targetVelocity " + i, motorTarget[i]);
                    packet.put("currentVelocity " + i, motors[i].getVelocity());
                    packet.put("currentPower" + i, motors[i].getPower());

                    dashboard.sendTelemetryPacket(packet);
                } else if (isPositionCloseLoop[i] && !isVelocityCloseLoop[i]) {
                    pidfControllers[i].setPIDF(kP[i], kI[i], kD[i], kF[i]);
                    double pos = motors[i].getCurrentPosition();

                    motors[i].setPower(pidfControllers[i].calculate(pos, motorTarget[i]));

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("targetPosition " + i, motorTarget[i]);
                    packet.put("currentPosition " + i, pos);

                    dashboard.sendTelemetryPacket(packet);
                } else if (!isPositionCloseLoop[i] && !isVelocityCloseLoop[i]) {
                    motors[i].setPower(motorTarget[i]);

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("currentVelocity " + i, motors[i].getVelocity());
                    packet.put("currentPosition " + i, motors[i].getCurrentPosition());

                    dashboard.sendTelemetryPacket(packet);
                }
            }

            if (!servoName[i].isEmpty()) {
                servos[i].setPosition(servoTarget[i]);
            }

            if (!colorSensorName[i].isEmpty()) {
                Color.RGBToHSV((int) (colorSensor[i].red() * SCALE_FACTOR),
                        (int) (colorSensor[i].green() * SCALE_FACTOR),
                        (int) (colorSensor[i].blue() * SCALE_FACTOR),
                        hsvValues);

                TelemetryPacket packet = new TelemetryPacket();

                packet.put("Alpha" + i, colorSensor[i].alpha());
                packet.put("Red" + i, colorSensor[i].red());
                packet.put("Green" + i, colorSensor[i].green());
                packet.put("Blue" + i, colorSensor[i].blue());
                packet.put("Hue" + i, hsvValues[0]);
                packet.put("Distance (cm)" + i, String.format(Locale.US, "%.02f", distanceSensor[i].getDistance(DistanceUnit.CM)));

                dashboard.sendTelemetryPacket(packet);
            }
        }
    }

    public void stopMotors() {
        for (DcMotorEx motor : motors) {
            if (motor != null) motor.setPower(0);
        }
    }
}
