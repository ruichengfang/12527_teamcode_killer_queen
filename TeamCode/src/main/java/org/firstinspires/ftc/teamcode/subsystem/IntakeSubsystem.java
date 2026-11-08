package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NameConstants;

/** Intake 与 Transit 的联合输送机构。 */
public class IntakeSubsystem {
    private final DcMotorEx intakeMotor;
    private final DcMotorEx transitMotor;

    private double intakePower;
    private double transitPower;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, NameConstants.INTAKE);
        transitMotor = hardwareMap.get(DcMotorEx.class, NameConstants.TRANSIT);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transitMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transitMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transitMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setPowers(double intakePower, double transitPower) {
        this.intakePower = intakePower;
        this.transitPower = transitPower;
        intakeMotor.setPower(intakePower);
        transitMotor.setPower(transitPower);
    }

    public void stop() {
        setPowers(0.0, 0.0);
    }

    public double getIntakePower() {
        return intakePower;
    }

    public double getTransitPower() {
        return transitPower;
    }
}
