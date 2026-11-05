package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.teamcode.subSystems.shooter;   // ← 已取消注释
import org.firstinspires.ftc.teamcode.subSystems.inTake;
import org.firstinspires.ftc.teamcode.subSystems.servo;
import org.firstinspires.ftc.teamcode.NameConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//声明向量结构r
class Vec{
    double dir,size;
};

@TeleOp(name = "Mecanum Drive", group = "Drive")
public class MecanumDrive extends LinearOpMode {

    public double normalizeAngle(double angle) {
        while (angle > 180)   angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    /**
     *常数部分-电机名称
     */
    String odName = NameConstants.OD;
//    String limelightName = NameConstants.LIMELIGHT;
    String leftFrontMotorName = NameConstants.LEFT_FRONT_MOTOR;
    String rightFrontMotorName = NameConstants.RIGHT_FRONT_MOTOR;
    String leftBackMotorName = NameConstants.LEFT_BACK_MOTOR;
    String rightBackMotorName = NameConstants.RIGHT_BACK_MOTOR;
    /**
     * 常数部分-整形系数/参数
     */
    double odErrorX = 20.1, odErrorY = -131.12;

    double x2Factor = 1.1;

    /**
     * 设备初始化声明
     */
    DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
//    AutoAim autoAim;
    inTake intake;
    servo servo;
//    shooter shooter;   // ← 新增成员声明

    @Override
    public void runOpMode() {

        GoBildaPinpointDriver pinpoint;
        Vec vec1 = new Vec(), vec2 = new Vec();
        double headingOffset = 0;//按三角键时记录当前角度

        intake = new inTake(hardwareMap);//intake初始化
        servo = new servo(hardwareMap);//servo初始化
//        shooter = new shooter(hardwareMap);

        //pinpoint 初始化
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, odName);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit distanceUnit;
        pinpoint.setOffsets(odErrorX, odErrorY, DistanceUnit.MM);  // 示例值，必须改成你的实际值
        pinpoint.resetPosAndIMU();
        pinpoint.update();

        //end of 初始化

        telemetry.addData("Device status", pinpoint.getDeviceStatus());
        telemetry.update();

        leftFrontMotor  = hardwareMap.get(DcMotor.class, leftFrontMotorName);
        rightFrontMotor = hardwareMap.get(DcMotor.class, rightFrontMotorName);
        leftBackMotor   = hardwareMap.get(DcMotor.class, leftBackMotorName);
        rightBackMotor  = hardwareMap.get(DcMotor.class, rightBackMotorName);

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

//        autoAim = new AutoAim();

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        // 将开始时的机器人方向设为舵机保持的初始方向
        servo.start();

        while (opModeIsActive()) {
//--------------------------底盘------------------------------------
            pinpoint.update();

            // 按三角键：把当前朝向设为新的"0°"
            if (gamepad1.left_stick_button) {
                headingOffset = pinpoint.getHeading(AngleUnit.RADIANS);
            }

            double y1  = -gamepad1.left_stick_y;
            double x1  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;
            double heading = pinpoint.getHeading(AngleUnit.RADIANS) - headingOffset; //修正heading

            while (heading > Math.PI)        heading -= 2 * Math.PI;
            while (heading <= -Math.PI)      heading += 2 * Math.PI;

            //vec1-遥控器移动向量 vec2-机器人移动向量
            //向量计算与变换
            vec1.dir =  Math.atan2(y1,x1);
            vec1.size = Math.sqrt(x1*x1+y1*y1);
            vec2.dir = vec1.dir - heading;
            vec2.size = vec1.size;

            /**
             * 自瞄：按下R2返回场地方向（0度）
             */

//            if (gamepad1.right_bumper) {
//                rx = autoAim.update(0, heading, servo);
//            }else
            rx = gamepad1.right_stick_x;

            telemetry.addData("heading deg", "%.1f", Math.toDegrees(heading));

            double x2 = vec2.size*Math.cos(vec2.dir) * x2Factor;
            double y2 = vec2.size*Math.sin(vec2.dir);

            //电机功率分配部分
            double fl = y2 + x2 + rx;
            double bl = y2 - x2 + rx;
            double fr = y2 - x2 - rx;
            double br = y2 + x2 - rx;

            double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                    Math.max(Math.abs(fr), Math.abs(br))));
            if (max > 1.0) {
                fl /= max;
                bl /= max;
                fr /= max;
                br /= max;
            }

            leftFrontMotor.setPower(fl);
            leftBackMotor.setPower(bl);
            rightFrontMotor.setPower(fr);
            rightBackMotor.setPower(br);



//========================= intake =============================

            if (gamepad1.right_trigger > 0.5)      intake.intake();
            else if (gamepad1.left_trigger > 0.5)  intake.outtake();
            else intake.constant();

//========================= servo ==============================

            servo.update(gamepad1.y, gamepad1.b, false);

//========================= shooter =============================

//            // 【新增】调用 shooter 子系统，传入所有手柄状态
//            shooter.update(getRuntime(),
//            gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.a,
//            gamepad1.right_bumper, gamepad1.b,
//            gamepad1.right_trigger, gamepad1.left_trigger);
//
//
//            telemetry.addData("shooter目标RPM", "%.0f", shooter.getTargetRPM());
//            telemetry.addData("shooter实际RPM", "%.0f", shooter.getFilteredRPM());
//            telemetry.addData("shooter功率01", "%.3f", shooter.getPower01());
//            telemetry.addData("shooter功率2", "%.3f", shooter.getPower2());
//            telemetry.addData("shooter功率3", "%.3f", shooter.getPower3());


            telemetry.update();

        }
    }
}