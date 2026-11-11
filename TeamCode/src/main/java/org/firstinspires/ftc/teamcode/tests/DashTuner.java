package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.DashboardTuningCommand;
import org.firstinspires.ftc.teamcode.subsystem.DashboardTuningSubsystem;

@TeleOp(name = "DashTuner")
@Config
public class DashTuner extends OpMode {
    public static String[] motorName = {"", "", "", ""};
    public static String[] servoName = {"", "", "", ""};
    public static double[] motorTarget = new double[4];
    public static double[] servoTarget = new double[4];
    public static int[] slaveTo = {-1, -1, -1, -1};
    public static boolean[] isPositionCloseLoop = new boolean[4];
    public static boolean[] isVelocityCloseLoop = new boolean[4];

    // PIDF parameters as static arrays for Dashboard configuration
    public static double[] kP = {0, 0, 0, 0};
    public static double[] kI = {0, 0, 0, 0};
    public static double[] kD = {0, 0, 0, 0};
    public static double[] kF = {0, 0, 0, 0};

    public static String[] colorSensorName = {"", "", "", ""};


    private DashboardTuningSubsystem tuning;
    private DashboardTuningCommand command;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        tuning = new DashboardTuningSubsystem(hardwareMap);
        command = new DashboardTuningCommand(tuning);
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(command);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
        tuning.stopMotors();
        CommandScheduler.getInstance().reset();
    }
}
