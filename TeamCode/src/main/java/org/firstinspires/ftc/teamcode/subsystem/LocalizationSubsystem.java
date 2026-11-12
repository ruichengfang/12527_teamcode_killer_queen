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
    public static final double OPTION_RESET_X_IN = 72.0;
    public static final double OPTION_RESET_Y_IN = 72.0;
    public static final double OPTION_RESET_HEADING_DEG = 180.0;

    // 遥操实测值，也是全车唯一的 pod 偏移来源；pedro/Constants.java 的
    // xPodOffset / yPodOffset 必须与此相同，改一处要同时改另一处。
    public static final double PINPOINT_X_OFFSET_IN = -0.787402;
    public static final double PINPOINT_Y_OFFSET_IN = -5.323543;

    private final IMU imu;
    private GoBildaPinpointDriver pinpoint;

    private boolean pinpointReady;
    private boolean needSetStartPosition;
    private boolean positionInitialized;
    private double pendingXIn;
    private double pendingYIn;
    private double pendingHeadingRad;
    private String pinpointStatus = "N/A";
    private double xIn;
    private double yIn;
    private double headingDeg;

    // 无头驾驶独立使用 Control Hub IMU，不依赖 Pinpoint 场地位姿。
    private double driverHeadingReferenceRadians = Double.NaN;

    public LocalizationSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG), false);
    }

    /** 自动使用：明确指定自己的起点，不读取此前保存的自动末位置。 */
    public LocalizationSubsystem(HardwareMap hardwareMap, double initialXIn,
                                 double initialYIn, double initialHeadingRad) {
        this(hardwareMap, initialXIn, initialYIn, initialHeadingRad, true);
    }

    /** Solo 保留设备已有位姿；指定起点的自动模式仍初始化 Pinpoint。 */
    private LocalizationSubsystem(HardwareMap hardwareMap, double initialXIn,
                                  double initialYIn, double initialHeadingRad, boolean initializePinpoint) {
        pendingXIn = initialXIn;
        pendingYIn = initialYIn;
        pendingHeadingRad = initialHeadingRad;
        needSetStartPosition = initializePinpoint;

        imu = hardwareMap.get(IMU.class, NameConstants.IMU);
        // Control Hub 安装在车右侧，Logo 朝右（车外），USB-C 接口朝上。
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, NameConstants.PINPOINT);
            if (initializePinpoint) {
                pinpoint.setEncoderResolution(
                        GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD);
                pinpoint.setOffsets(PINPOINT_X_OFFSET_IN, PINPOINT_Y_OFFSET_IN, DistanceUnit.INCH);
                pinpoint.resetPosAndIMU();
            }
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
        }

        Pose2D pose = pinpoint.getPosition();
        xIn = pose.getX(DistanceUnit.INCH);
        yIn = pose.getY(DistanceUnit.INCH);
        headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
        positionInitialized = Double.isFinite(xIn) && Double.isFinite(yIn) && Double.isFinite(headingDeg);
    }

    /** 将当前 IMU 航向记为驾驶员前方；无效读数不会覆盖已有参考。 */
    public void resetDriverHeading() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (Double.isFinite(yaw)) driverHeadingReferenceRadians = yaw;
    }

    /** 内置 IMU 相对驾驶员前方的航向；无有效 IMU 读数时返回 NaN。 */
    public double getDriverHeadingRadians() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (!Double.isFinite(yaw)) return Double.NaN;
        if (!Double.isFinite(driverHeadingReferenceRadians)) driverHeadingReferenceRadians = yaw;
        return Math.IEEEremainder(yaw - driverHeadingReferenceRadians, 2.0 * Math.PI);
    }

    /** 手动无头驾驶航向由内置 IMU 提供，与 Pinpoint 状态无关。 */
    public double getManualDriveHeadingRadians() {
        return getDriverHeadingRadians();
    }

    /** 恢复手动默认起点；Solo 左摇杆按下只重设驾驶员前方，不调用此方法。 */
    public void reset() {
        imu.resetYaw();
        resetDriverHeading();
        if (pinpointReady) {
            pendingXIn = START_X_IN;
            pendingYIn = START_Y_IN;
            pendingHeadingRad = Math.toRadians(START_HEADING_DEG);
            needSetStartPosition = true;
            positionInitialized = false;
            update();
        }
    }

    /** Option：将场地位姿写为 (72 in, 72 in, 180°)，等待 Pinpoint READY 后生效。 */
    public boolean resetToFieldCenter() {
        if (!pinpointReady) return false;
        pendingXIn = OPTION_RESET_X_IN;
        pendingYIn = OPTION_RESET_Y_IN;
        pendingHeadingRad = Math.toRadians(OPTION_RESET_HEADING_DEG);
        needSetStartPosition = true;
        positionInitialized = false;
        update();
        return true;
    }

    /** 在当前有效位姿基础上平移坐标，保持场地航向和驾驶员前方。 */
    public boolean adjustFieldPosition(double deltaXIn, double deltaYIn) {
        if (!isFieldPoseValid() || !Double.isFinite(deltaXIn) || !Double.isFinite(deltaYIn)
                || !Double.isFinite(xIn + deltaXIn) || !Double.isFinite(yIn + deltaYIn)) return false;
        pendingXIn = xIn + deltaXIn;
        pendingYIn = yIn + deltaYIn;
        pendingHeadingRad = Math.toRadians(headingDeg);
        needSetStartPosition = true;
        positionInitialized = false;
        update();
        return true;
    }

    private void setPinpointPosition(double xIn, double yIn, double headingRad) {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, xIn, yIn,
                AngleUnit.RADIANS, headingRad));
        this.xIn = xIn;
        this.yIn = yIn;
        headingDeg = Math.toDegrees(headingRad);
    }

    /** 返回场地航向供遥测与云台使用；初始化前返回 Control Hub IMU 航向。 */
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

    /** 射击用场地位姿有效：Pinpoint 已连接、本周期 READY 且已读到有限坐标。 */
    public boolean isFieldPoseValid() {
        return pinpointReady && positionInitialized && "READY".equals(pinpointStatus)
                && Double.isFinite(xIn) && Double.isFinite(yIn) && Double.isFinite(headingDeg);
    }

    /** 当前 Pinpoint 场地位姿（英寸 / 度）。 */
    public Pose2D getFieldPose() {
        return new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg);
    }

    /** 对应 OT Drive.getTranslationSpeedInchesPerSecond；位姿无效时为 NaN。 */
    public double getTranslationSpeedInchesPerSecond() {
        return isFieldPoseValid() ? Math.hypot(
                pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH)) : Double.NaN;
    }
}
