import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.hardware.*;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.command.*;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.tests.DashTuner;
import java.lang.reflect.*;
import java.util.*;

/** Independent expectations supplement comparison with the old implementation. */
public final class LifecycleCheck {
    static void check(boolean condition,String message) {
        if(!condition)throw new AssertionError(message);
    }
    static void close(double actual,double expected,String label) {
        check(Math.abs(actual-expected)<1e-10,label+": "+actual+" != "+expected);
    }
    static Object field(Object object,String name)throws Exception {
        Field f=object.getClass().getDeclaredField(name);f.setAccessible(true);return f.get(object);
    }
    static CommandScheduler scheduler(){return CommandScheduler.getInstance();}
    static void allMotorsStopped(HardwareMap hardware) {
        for(String name:BehaviorReplay.MOTORS)
            close(hardware.get(DcMotorEx.class,name).power,0,"stopped "+name);
    }

    static void shootingAndOwnership()throws Exception {
        AutoEndPose.clear();
        Teleop op=new Teleop();op.hardwareMap=BehaviorReplay.hardware(false);op.init();op.start();
        TeleopControlCommand controls=(TeleopControlCommand)field(op,"controls");
        check(controls.getRequirements().size()==6,"parent must own all six subsystems");
        op.gamepad1.b=true;op.clock=1;op.loop();
        close(op.hardwareMap.get(DcMotorEx.class,NameConstants.SHOOTER_0).power,-1,"spinup output");
        close(op.hardwareMap.get(DcMotorEx.class,NameConstants.INTAKE).power,0,"no early feed");
        op.clock=1.099;op.loop();
        close(op.hardwareMap.get(DcMotorEx.class,NameConstants.INTAKE).power,0,"100ms boundary");
        op.clock=1.1;op.loop();
        close(op.hardwareMap.get(DcMotorEx.class,NameConstants.INTAKE).power,1,"feed even with no pinpoint");
        close(op.hardwareMap.get(Servo.class,NameConstants.BLOCKER).position,.55,"open blocker");
        op.gamepad1.start=true;op.clock=2.1;op.loop();
        close(op.hardwareMap.get(DcMotorEx.class,NameConstants.INTAKE).power,1,"Start no longer resets");
        op.gamepad1.left_stick_button=true;op.clock=2.11;op.loop();
        allMotorsStopped(op.hardwareMap);
        op.clock=2.12;op.loop();
        close(op.hardwareMap.get(DcMotorEx.class,NameConstants.SHOOTER_0).power,0,
            "held B must not resume after left-stick reset");
        op.gamepad1.b=false;op.gamepad1.left_stick_button=false;op.clock=2.2;op.loop();
        op.gamepad1.b=true;op.gamepad1.left_stick_y=-1;op.clock=3;op.loop();
        DriveSubsystem drive=(DriveSubsystem)field(op,"drive");
        close(drive.getFrontLeftPower(),0,"B suppresses joystick driving");
        check(op.hardwareMap.get(DcMotorEx.class,NameConstants.FRONT_LEFT).direction==DcMotor.Direction.REVERSE,
            "jrc left motor direction");
        check(op.hardwareMap.get(DcMotorEx.class,NameConstants.FRONT_RIGHT).direction==DcMotor.Direction.FORWARD,
            "jrc right motor direction");
        // A conflicting command must cancel the WHOLE composition, including the B child.
        Command takeover=new RunCommand(()->{},drive);
        scheduler().schedule(takeover);
        check(!scheduler().isScheduled(controls),"resource conflict cancels parent");
        allMotorsStopped(op.hardwareMap);
        close(op.hardwareMap.get(Servo.class,NameConstants.BLOCKER).position,.38,"cancel closes blocker");
        check(!op.gamepad1.rumbling,"cancel stops rumble");
        scheduler().cancelAll();scheduler().reset();
    }

    static void pathSequence() {
        scheduler().reset();Follower follower=new Follower();
        int[] snapshots={0};
        PathFollowerSubsystem drive=new PathFollowerSubsystem(follower,()-> {
            check(follower.updates==snapshots[0]+1,"save occurs immediately after follower update");
            snapshots[0]++;
        });
        Command routine=new SequentialCommandGroup(
            new FollowPathCommand(drive,new Path("1")),
            new FollowPathCommand(drive,new Path("2")),
            new FollowPathCommand(drive,new Path("3")));
        scheduler().schedule(routine);
        check(follower.events.equals(Arrays.asList("follow:1")),"start first path");
        scheduler().run();check(follower.updates==1,"one follower update per tick");
        follower.atEnd=true;scheduler().run();
        check(follower.events.equals(Arrays.asList("follow:1","follow:2")),"same-cycle path handoff");
        follower.atEnd=true;scheduler().run();follower.atEnd=true;scheduler().run();
        check(!scheduler().isScheduled(routine),"routine finished");
        check(follower.events.equals(Arrays.asList("follow:1","follow:2","follow:3")),
            "normal completion must leave Pedro holding endpoint");
        // SequentialCommandGroup and its children are reusable.
        scheduler().schedule(routine);scheduler().cancel(routine);
        check(follower.events.subList(3,follower.events.size()).equals(
            Arrays.asList("follow:1","stop","motorStop")),"cancel stops follower AND drivetrain");
        scheduler().reset();
    }

    static void timedDrive() {
        scheduler().reset();HardwareMap hardware=BehaviorReplay.hardware(true);
        DriveSubsystem drive=new DriveSubsystem(hardware);double[] time={0};
        TimedDriveCommand command=new TimedDriveCommand(drive,.7,.2,()->time[0]);
        scheduler().schedule(command);
        close(drive.getFrontLeftPower(),.7,"demo power");
        time[0]=.199;scheduler().run();check(scheduler().isScheduled(command),"not before 200ms");
        time[0]=.2;scheduler().run();check(!scheduler().isScheduled(command),"finish at 200ms");
        allMotorsStopped(hardware);
        scheduler().schedule(command);scheduler().cancelAll();allMotorsStopped(hardware);
        scheduler().reset();
    }

    static void leafCommands() {
        scheduler().reset();HardwareMap hardware=BehaviorReplay.hardware(true);
        BlockerSubsystem blocker=new BlockerSubsystem(hardware);Gamepad gamepad=new Gamepad();
        BlockerCommand command=new BlockerCommand(gamepad,blocker,()->true,()->false);
        scheduler().schedule(command);scheduler().run();
        close(hardware.get(Servo.class,NameConstants.BLOCKER).position,.39,"standalone blocker writes output");
        check(command.getRequirements().contains(blocker),"blocker requirement");
        scheduler().cancelAll();scheduler().reset();
        LocalizationSubsystem localization=new LocalizationSubsystem(hardware);
        TurretSubsystem turret=new TurretSubsystem(hardware);
        turret.toggleAimMode(true);
        Command reset=new ResetLocalizationCommand(localization,turret);
        scheduler().schedule(reset);scheduler().run();
        check(!scheduler().isScheduled(reset)&&!turret.isAimMode(),"reset is one-shot");
        scheduler().reset();
    }

    static void tunerStop() {
        DashTuner.motorName=new String[]{NameConstants.FRONT_LEFT,"","",""};
        DashTuner.servoName=new String[]{"","","",""};
        DashTuner.colorSensorName=new String[]{"","","",""};
        DashTuner.motorTarget=new double[]{.6,0,0,0};
        DashTuner.isPositionCloseLoop=new boolean[4];DashTuner.isVelocityCloseLoop=new boolean[4];
        DashTuner.slaveTo=new int[]{-1,-1,-1,-1};
        DashTuner op=new DashTuner();op.hardwareMap=BehaviorReplay.hardware(false);
        op.init();op.start();op.loop();
        close(op.hardwareMap.get(DcMotorEx.class,NameConstants.FRONT_LEFT).power,.6,"tuner runs");
        op.stop();allMotorsStopped(op.hardwareMap);
    }

    static void poseHandoff() {
        scheduler().reset();AutoEndPose.clear();
        AutoEndPose.save(24,48,Math.toRadians(135));
        AutoEndPose.save(Double.NaN,0,0);
        close(AutoEndPose.getXIn(),24,"invalid pose must not overwrite saved pose");
        HardwareMap hardware=BehaviorReplay.hardware(true);
        com.qualcomm.hardware.gobilda.GoBildaPinpointDriver od=hardware.get(
            com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class,NameConstants.PINPOINT);
        od.status=com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        LocalizationSubsystem localization=new LocalizationSubsystem(hardware);
        localization.update();check(!localization.isPositionInitialized(),"wait for READY");
        od.status=com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.READY;
        localization.update();check(localization.isPositionInitialized(),"restore on READY");
        close(localization.getXIn(),24,"restore x");close(localization.getYIn(),48,"restore y");
        close(localization.getYawDegrees(),135,"use saved field heading");
        close(localization.getImuYawDegrees(),0,"IMU remains independent");
        close(od.ox,.787402,"jrc x offset inches");close(od.oy,5.323543,"jrc y offset inches");
        localization.reset();close(localization.getXIn(),8,"manual reset x");
        close(localization.getYIn(),111,"manual reset y");close(localization.getYawDegrees(),0,"reset heading");
        close(AutoEndPose.getXIn(),24,"manual reset must not clear auto record");
        LocalizationSubsystem explicit=new LocalizationSubsystem(hardware,1,2,Math.toRadians(90));
        explicit.update();close(explicit.getXIn(),1,"auto ignores saved pose");
        close(explicit.getYawDegrees(),90,"explicit radians converted correctly");
        AutoEndPose.clear();check(!AutoEndPose.hasValue(),"clear before next auto");
        scheduler().reset();
    }

    static void fixedTurretRemainsOptIn() {
        scheduler().reset();
        check(!TurretSubsystem.FIXED_POSITION_ENABLED,"fixed mode must remain disabled by default");
        HardwareMap hardware=BehaviorReplay.hardware(true);
        TurretSubsystem defaultTurret=new TurretSubsystem(hardware);
        check(!defaultTurret.isFixedPositionEnabled(),"production constructor keeps old behavior");
        check(defaultTurret.setAimMode(true,true),"normal aim still enabled");
        scheduler().reset();
        hardware=BehaviorReplay.hardware(true);
        LocalizationSubsystem localization=new LocalizationSubsystem(hardware);
        TurretSubsystem turret=new TurretSubsystem(hardware,true);
        check(turret.isFixedPositionEnabled(),"explicit test instance locked");
        close(hardware.get(Servo.class,NameConstants.TURRET_LEFT).position,.5,"fixed INIT left");
        close(hardware.get(Servo.class,NameConstants.TURRET_RIGHT).position,.5,"fixed INIT right");
        ShooterSubsystem shooter=new ShooterSubsystem(hardware);
        IntakeSubsystem intake=new IntakeSubsystem(hardware);
        BlockerSubsystem blocker=new BlockerSubsystem(hardware);
        double[] time={0};
        AimAndShootCommand aim=new AimAndShootCommand(shooter,turret,blocker,intake,localization,()->time[0]);
        aim.initialize();
        for(int i=0;i<100;i++) {
            hardware.get(IMU.class,NameConstants.IMU).yaw=i*17;
            localization.update();
            turret.toggleAimMode(true);turret.setAimMode(true,true);
            turret.toggleLockMode(i*17);turret.adjustLeft();turret.adjustRight();
            turret.reset();turret.update(localization);
            time[0]=i*.02;aim.execute();
            close(hardware.get(Servo.class,NameConstants.TURRET_LEFT).position,.5,"locked left");
            close(hardware.get(Servo.class,NameConstants.TURRET_RIGHT).position,.5,"locked right");
            check(!turret.isAimMode()&&!turret.isLockMode(),"all aim/heading toggles ignored");
        }
        close(intake.getIntakePower(),1,"fixed turret must not disable timed feeding");
        aim.end(true);scheduler().reset();
    }

    public static void main(String[] args)throws Exception {
        shootingAndOwnership();pathSequence();timedDrive();leafCommands();tunerStop();
        poseHandoff();fixedTurretRemainsOptIn();
        System.out.println("PASS: resource arbitration, 100ms shot timing, left-stick reset, pose handoff, cancellation, path handoff/hold, demo timing, leaf lifecycle, tuner STOP, disabled-by-default turret lock");
    }
}
