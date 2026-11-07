package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.NameConstants;

/** 麦轮底盘及其 Pinpoint 定位。 */
public class DriveSubsystem {
    private static final double X_FACTOR = 1.1;
    private final DcMotor leftFront, leftBack, rightFront, rightBack;
    private final GoBildaPinpointDriver pinpoint;
    private double headingOffset;

    public DriveSubsystem(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, NameConstants.OD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(-20, -135.2, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
        pinpoint.update();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 72, 72,
                AngleUnit.DEGREES, 0));
        pinpoint.update();

        leftFront = hardwareMap.get(DcMotor.class, NameConstants.LEFT_FRONT_MOTOR);
        leftBack = hardwareMap.get(DcMotor.class, NameConstants.LEFT_BACK_MOTOR);
        rightFront = hardwareMap.get(DcMotor.class, NameConstants.RIGHT_FRONT_MOTOR);
        rightBack = hardwareMap.get(DcMotor.class, NameConstants.RIGHT_BACK_MOTOR);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void updateHeading() { pinpoint.update(); }

    public void resetFieldHeading() {
        headingOffset = pinpoint.getHeading(AngleUnit.RADIANS);
    }

    private double relativeHeading() {
        double heading = pinpoint.getHeading(AngleUnit.RADIANS) - headingOffset;
        while (heading > Math.PI) heading -= 2 * Math.PI;
        while (heading <= -Math.PI) heading += 2 * Math.PI;
        return heading;
    }

    public double getRelativeHeadingDegrees() { return Math.toDegrees(relativeHeading()); }

    public Object getDeviceStatus() { return pinpoint.getDeviceStatus(); }

    public void driveFieldCentric(double x, double y, double rotation) {
        double direction = Math.atan2(y, x) - relativeHeading();
        double magnitude = Math.hypot(x, y);
        double x2 = magnitude * Math.cos(direction) * X_FACTOR;
        double y2 = magnitude * Math.sin(direction);
        double fl = y2 + x2 + rotation;
        double bl = y2 - x2 + rotation;
        double fr = y2 - x2 - rotation;
        double br = y2 + x2 - rotation;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));
        leftFront.setPower(fl / max);
        leftBack.setPower(bl / max);
        rightFront.setPower(fr / max);
        rightBack.setPower(br / max);
    }

    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
