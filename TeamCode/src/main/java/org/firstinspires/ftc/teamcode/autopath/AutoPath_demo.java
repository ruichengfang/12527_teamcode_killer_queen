package org.firstinspires.ftc.teamcode.autopath;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AutoEndPose;
import org.firstinspires.ftc.teamcode.command.TimedDriveCommand;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

/** 四轮以 0.7 功率前进 0.2 秒；保留 jrc 定位继承，并由命令统一结束/取消。 */
@Autonomous(name = "AutoPath_demo", group = "Autonomous")
public class AutoPath_demo extends LinearOpMode {
    private static final double FORWARD_POWER = 0.7;
    private static final double DRIVE_TIME_SECONDS = 0.2;
    private LocalizationSubsystem localization;

    @Override
    public void runOpMode() {
        AutoEndPose.clear();
        CommandScheduler.getInstance().reset();
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        drive.stop();
        localization = new LocalizationSubsystem(hardwareMap,
                LocalizationSubsystem.START_X_IN, LocalizationSubsystem.START_Y_IN,
                Math.toRadians(LocalizationSubsystem.START_HEADING_DEG));
        Command motion = new TimedDriveCommand(drive, FORWARD_POWER, DRIVE_TIME_SECONDS,
                this::getRuntime);
        boolean autoStarted = false;
        try {
            while (!isStarted() && !isStopRequested()) {
                localization.update();
                showPose("等待开始；启动后前进 0.2 秒");
                sleep(20);
            }
            if (isStopRequested()) return;
            autoStarted = true;
            CommandScheduler.getInstance().schedule(motion);
            while (opModeIsActive() && CommandScheduler.getInstance().isScheduled(motion)) {
                CommandScheduler.getInstance().run();
                if (!CommandScheduler.getInstance().isScheduled(motion)) break;
                localization.update();
                saveCurrentPose();
                showPose("前进中");
                sleep(5);
            }
        } finally {
            CommandScheduler.getInstance().cancelAll();
            drive.stop();
            if (autoStarted) {
                localization.update();
                saveCurrentPose();
            }
            CommandScheduler.getInstance().reset();
        }
    }

    private void saveCurrentPose() {
        if (localization.isPositionInitialized()) {
            AutoEndPose.save(localization.getXIn(), localization.getYIn(),
                    Math.toRadians(localization.getHeadingDeg()));
        }
    }

    private void showPose(String state) {
        telemetry.addData("状态", state);
        telemetry.addData("Pinpoint", localization.getPinpointStatus());
        if (localization.isPositionInitialized()) {
            telemetry.addData("x (in)", localization.getXIn());
            telemetry.addData("y (in)", localization.getYIn());
            telemetry.addData("heading (deg)", localization.getHeadingDeg());
            telemetry.addData("坐标遗传", AutoEndPose.hasValue() ? "当前位置已保存" : "等待有效定位");
        } else {
            telemetry.addLine("等待 Pinpoint READY 并写入起点");
        }
        telemetry.update();
    }
}
