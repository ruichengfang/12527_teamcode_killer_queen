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
    private boolean needSetStartPosition = true;
    private boolean positionInitialized;
    private double pendingXIn;
    private double pendingYIn;
    private double pendingHeadingRad;
    private double pendingPreviousHeadingRad = Double.NaN;
    private String pinpointStatus = "N/A";
    private double xIn;
    private double yIn;
    private double headingDeg;

    /** 手动使用：有自动记录则恢复自动末位置，否则使用默认起点。 */
    // ---- OT Drive 无头驾驶参考（移植自 Overtune Drive.java，逻辑不变）----
    /** OT DRIVER_HEADING_HOLD_NANOS：Pinpoint 短暂坏读时沿用上一帧有效角度的时长。 */
    private static final long DRIVER_HEADING_HOLD_NANOS = 100_000_000L;
    private double driverHeadingReferenceRadians = Double.NaN;
    private boolean periodicBadRead;
    private double lastValidDriverHeadingRadians = Double.NaN;
    private long lastValidDriverHeadingNanos;

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

        long readStarted = System.nanoTime();
        pinpoint.update();
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        pinpointStatus = status.toString();
        periodicBadRead = status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ;

        // 复位尚未完成时不能写入或读取有效位置。
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            return;
        }

        if (needSetStartPosition) {
            if (!Double.isFinite(pendingPreviousHeadingRad)) {
                pendingPreviousHeadingRad = Math.toRadians(headingDeg);
            }
            setPinpointPosition(pendingXIn, pendingYIn, pendingHeadingRad);
            pinpoint.update();
            pinpointStatus = pinpoint.getDeviceStatus().toString();
            if (!"READY".equals(pinpointStatus)) {
                return;
            }
            // 改写场地航向时保留驾驶员前方；读回失败重试也只调整一次。
            if (Double.isFinite(driverHeadingReferenceRadians) && Double.isFinite(pendingPreviousHeadingRad)) {
                driverHeadingReferenceRadians = Math.IEEEremainder(driverHeadingReferenceRadians
                        + (pendingHeadingRad - pendingPreviousHeadingRad), 2.0 * Math.PI);
            }
            pendingPreviousHeadingRad = Double.NaN;
            needSetStartPosition = false;
            positionInitialized = true;
        }

        Pose2D pose = pinpoint.getPosition();
        xIn = pose.getX(DistanceUnit.INCH);
        yIn = pose.getY(DistanceUnit.INCH);
        headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
        // OT Drive.periodic：参考丢失后在第一帧有效读数时重新取当前朝向为前方。
        if (!Double.isFinite(driverHeadingReferenceRadians)) {
            resetDriverHeading();
        }
        lastValidDriverHeadingRadians = getDriverHeadingRadians();
        lastValidDriverHeadingNanos = readStarted;
    }

    /** OT Drive.resetDriverHeading：当前车头朝向记为驾驶员前方（START 和左摇杆按下时调用）。 */
    public void resetDriverHeading() {
        driverHeadingReferenceRadians = isFieldPoseValid() ? Math.toRadians(headingDeg) : Double.NaN;
        lastValidDriverHeadingRadians = Double.NaN;
    }

    /** OT Drive.getDriverHeadingRadians：Pinpoint 不可用时返回 NaN。 */
    public double getDriverHeadingRadians() {
        if (!isFieldPoseValid() || !Double.isFinite(driverHeadingReferenceRadians)) {
            return Double.NaN;
        }
        return Math.IEEEremainder(Math.toRadians(headingDeg) - driverHeadingReferenceRadians, 2.0 * Math.PI);
    }

    /**
     * OT Drive.moveFieldCentric 使用的角度：正常时为驾驶员相对角；短暂坏读（≤0.1 s）沿用上一帧；
     * 否则 NaN，底盘退回车头坐标驾驶。
     */
    public double getManualDriveHeadingRadians() {
        double heading = getDriverHeadingRadians();
        if (!Double.isFinite(heading) && periodicBadRead && pinpointReady
                && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ
                && System.nanoTime() - lastValidDriverHeadingNanos <= DRIVER_HEADING_HOLD_NANOS) {
            heading = lastValidDriverHeadingRadians;
        }
        return heading;
    }

    /** 左摇杆按下仍恢复手动默认起点，不修改已保存的自动末位置。 */
    public void reset() {
        imu.resetYaw();
        if (pinpointReady) {
            pendingXIn = START_X_IN;
            pendingYIn = START_Y_IN;
            pendingHeadingRad = Math.toRadians(START_HEADING_DEG);
            pendingPreviousHeadingRad = Double.NaN;
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
        pendingPreviousHeadingRad = Double.NaN;
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
        pendingPreviousHeadingRad = Double.NaN;
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

    /** 射击用场地位姿有效：Pinpoint 已配置、已写入起点、本周期 READY 且坐标有限。 */
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
