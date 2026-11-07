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

    /** 设置串联进料功率，正值进料、负值退料。 */
    public void setPower(double power) {
        intakeMotor.setPower(power);
        transitMotor.setPower(-power);
    }

    /** 只驱动 intake，transit 保持停止。 */
    public void setIntakeOnly(double power) {
        intakeMotor.setPower(power);
        transitMotor.setPower(0.0);
    }

    /** 停止 intake 和 transit，供自动程序结束时调用。 */
    public void stop() {
        setPower(0.0);
    }
}
