package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutoEndPose;
import org.firstinspires.ftc.teamcode.NameConstants;

/** 集中管理 Control Hub IMU 与 GoBilda Pinpoint 定位。 */
public class LocalizationSubsystem extends SubsystemBase {
    public static final double START_X_IN = 8.0;
    public static final double START_Y_IN = 111.0;
    public static final double START_HEADING_DEG = 0.0;

    // 遥操实测值，也是全车唯一的 pod 偏移来源；pedro/Constants.java 的
    // xPodOffset / yPodOffset 必须与此相同，改一处要同时改另一处。
    public static final double PINPOINT_X_OFFSET_IN = 0.787402;
    public static final double PINPOINT_Y_OFFSET_IN = 5.323543;

    private final IMU imu;
    private GoBildaPinpointDriver pinpoint;

    private boolean pinpointReady;
    private boolean needSetStartPosition = true;
    private boolean positionInitialized;
    private double pendingXIn;
    private double pendingYIn;
    private double pendingHeadingRad;
    private String pinpointStatus = "N/A";
    private double xIn;
    private double yIn;
    private double headingDeg;

    /** 手动使用：有自动记录则恢复自动末位置，否则使用默认起点。 */
    public LocalizationSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap,
                AutoEndPose.hasValue() ? AutoEndPose.getXIn() : START_X_IN,
                AutoEndPose.hasValue() ? AutoEndPose.getYIn() : START_Y_IN,
                AutoEndPose.hasValue() ? AutoEndPose.getHeadingRad() : Math.toRadians(START_HEADING_DEG));
    }

    /** 自动使用：明确指定自己的起点，不读取此前保存的自动末位置。 */
    public LocalizationSubsystem(HardwareMap hardwareMap, double initialXIn,
                                 double initialYIn, double initialHeadingRad) {
        pendingXIn = initialXIn;
        pendingYIn = initialYIn;
        pendingHeadingRad = initialHeadingRad;

        imu = hardwareMap.get(IMU.class, NameConstants.IMU);
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, NameConstants.PINPOINT);
            pinpoint.setEncoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.setOffsets(PINPOINT_X_OFFSET_IN, PINPOINT_Y_OFFSET_IN, DistanceUnit.INCH);
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

        // 复位尚未完成时不能写入或读取有效位置。
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            return;
        }

        if (needSetStartPosition) {
            setPinpointPosition(pendingXIn, pendingYIn, pendingHeadingRad);
            pinpoint.update();
            pinpointStatus = pinpoint.getDeviceStatus().toString();
            if (!"READY".equals(pinpointStatus)) {
                return;
            }
            needSetStartPosition = false;
            positionInitialized = true;
        }

        Pose2D pose = pinpoint.getPosition();
        xIn = pose.getX(DistanceUnit.INCH);
        yIn = pose.getY(DistanceUnit.INCH);
        headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    /** 左摇杆按下仍恢复手动默认起点，不修改已保存的自动末位置。 */
    public void reset() {
        imu.resetYaw();
        if (pinpointReady) {
            pendingXIn = START_X_IN;
            pendingYIn = START_Y_IN;
            pendingHeadingRad = Math.toRadians(START_HEADING_DEG);
            needSetStartPosition = true;
            positionInitialized = false;
            update();
        }
    }

    private void setPinpointPosition(double xIn, double yIn, double headingRad) {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, xIn, yIn,
                AngleUnit.RADIANS, headingRad));
        this.xIn = xIn;
        this.yIn = yIn;
        headingDeg = Math.toDegrees(headingRad);
    }

    /** 底盘和云台统一使用场地航向；Pinpoint 不可用时才回退到 Control Hub IMU。 */
    public double getYawRadians() {
        if (positionInitialized) {
            return Math.toRadians(headingDeg);
        }
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getYawDegrees() {
        return Math.toDegrees(getYawRadians());
    }

    public double getImuYawDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public boolean isPinpointReady() {
        return pinpointReady;
    }

    public boolean isPositionInitialized() {
        return positionInitialized;
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
