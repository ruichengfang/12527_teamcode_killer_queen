package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NameConstants;

public class IntakeSubsystem {
    private String intakeName = NameConstants.INTAKE_MOTOR;
    private String transitName =  NameConstants.TRANSIT_MOTOR;

    private DcMotor intakeMotor;   // private：外部不许直接碰
    private DcMotor transitMotor;

    // 构造器：new 的时候传入 hardwareMap
    public IntakeSubsystem(HardwareMap hw) {
        intakeMotor = hw.get(DcMotor.class, intakeName);
        transitMotor = hw.get(DcMotor.class, transitName);
    }

    // 对外只暴露"意图"，隐藏实现
    public void intake()  {
        intakeMotor.setPower(1.0);
        transitMotor.setPower(-1.0);
    }

    public void outtake() {
        intakeMotor.setPower(-1.0);
        transitMotor.setPower(1.0);
    }

    public void constant() {
        intakeMotor.setPower(0.4);
    }
}
