package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.NameConstants;

/** 集中管理 Control Hub IMU 与 GoBilda Pinpoint 定位。 */
public class LocalizationSubsystem extends SubsystemBase {
    public static final double START_X_IN = 8.0;
    public static final double START_Y_IN = 111.0;
    public static final double START_HEADING_DEG = 0.0;

    private static final double PINPOINT_X_OFFSET_MM = 20.0;
    private static final double PINPOINT_Y_OFFSET_MM = -135.2;

    private final IMU imu;
    private GoBildaPinpointDriver pinpoint;

    private boolean pinpointReady;
    private boolean needSetStartPosition = true;
    private String pinpointStatus = "N/A";
    private double xIn;
    private double yIn;
    private double headingDeg;

    public LocalizationSubsystem(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, NameConstants.IMU);
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, NameConstants.PINPOINT);
            pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();
            pinpointReady = true;
        } catch (Exception exception) {
            pinpoint = null;
            pinpointReady = false;
        }
    }

    public void update() {
        if (!pinpointReady) {
            return;
        }

        pinpoint.update();
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        pinpointStatus = status.toString();

        if (needSetStartPosition && status == GoBildaPinpointDriver.DeviceStatus.READY) {
            setPinpointStartPosition();
            needSetStartPosition = false;
        }

        Pose2D pose = pinpoint.getPosition();
        xIn = pose.getX(DistanceUnit.INCH);
        yIn = pose.getY(DistanceUnit.INCH);
        headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public void reset() {
        imu.resetYaw();
        if (pinpointReady) {
            setPinpointStartPosition();
            needSetStartPosition = false;
        }
    }

    private void setPinpointStartPosition() {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, START_X_IN, START_Y_IN,
                AngleUnit.DEGREES, START_HEADING_DEG));
        xIn = START_X_IN;
        yIn = START_Y_IN;
        headingDeg = START_HEADING_DEG;
    }

    public double getYawRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getYawDegrees() {
        return Math.toDegrees(getYawRadians());
    }

    public boolean isPinpointReady() {
        return pinpointReady;
    }

    public String getPinpointStatus() {
        return pinpointReady ? pinpointStatus : "未配置";
    }

    public double getXIn() {
        return xIn;
    }

    public double getYIn() {
        return yIn;
    }

    public double getHeadingDeg() {
        return headingDeg;
    }
}
