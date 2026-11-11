import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.command.*;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.subsystem.DashboardTuner;
import org.firstinspires.ftc.teamcode.shoot.*;
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

    static void tick(Teleop op,double t){op.clock=t;com.qualcomm.robotcore.util.ElapsedTime.now=t;op.loop();}
    static double power(HardwareMap h,String n){return h.get(DcMotorEx.class,n).power;}
    static double servo(HardwareMap h,String n){return h.get(Servo.class,n).position;}

    /** Ported Overtune ShootAssistCommand on hold-B: gate, continuous feed, jam, invalid target, release. */
    static void shootingAndOwnership()throws Exception {
        AutoEndPose.clear();scheduler().reset();
        Teleop op=new Teleop();op.hardwareMap=BehaviorReplay.hardware(true);op.init();op.start();
        HardwareMap hw=op.hardwareMap;
        TeleopControlCommand controls=(TeleopControlCommand)field(op,"controls");
        check(controls.getRequirements().size()==6,"parent must own all six subsystems");
        GoBildaPinpointDriver od=hw.get(GoBildaPinpointDriver.class,NameConstants.PINPOINT);
        DcMotorEx fly=hw.get(DcMotorEx.class,NameConstants.SHOOTER_0);
        tick(op,0);   // localization writes start pose
        // Robot at (100,84) heading 0: target (60,84) lies directly behind -> rear shooter aligned (OT 180 deg).
        od.pose=new Pose2D(DistanceUnit.INCH,100,84,AngleUnit.DEGREES,0);
        double t=0.02;
        op.gamepad1.b=true;tick(op,t);
        ShootAssistCommand assist=controls.getShootAssist();
        close(controls.getAimTarget().getYawErrorDegrees(),0,"rear-facing yaw error");
        close(controls.getAimTarget().getTargetYInches(),84,"upper-half aim point");
        // Like OT (Shooter.periodic before commands) the PID writes the new target next cycle.
        t+=.02;tick(op,t);
        check(fly.power<0,"flywheel spins up on B");
        // Aim locks but flywheel speed is not ready -> no gate, no transit feed.
        for(int i=0;i<20;i++){t+=.02;tick(op,t);}
        check(controls.getAutoAim().isLocked(),"aligned chassis locks after stable time");
        check(!((ShooterSubsystem)field(op,"shooter")).isReady(),"stalled flywheel not ready");
        check(assist.getFiringState()==ShotTransit.FiringState.WAITING,"waiting before speed ready");
        close(power(hw,NameConstants.TRANSIT),0,"no transit feed before speed ready");
        close(servo(hw,NameConstants.BLOCKER),.38,"gate closed before speed ready");
        // Position hold: displacement while no stick input drives back toward the anchor.
        od.pose=new Pose2D(DistanceUnit.INCH,102,84,AngleUnit.DEGREES,0);
        t+=.02;tick(op,t);
        check(power(hw,NameConstants.FRONT_LEFT)<0&&power(hw,NameConstants.BACK_RIGHT)<0,
            "position hold drives back toward anchor");
        od.pose=new Pose2D(DistanceUnit.INCH,100,84,AngleUnit.DEGREES,0);
        // Speed ready (3300 RPM == 1540 ticks/s) -> gate opens, brief reverse, then feed.
        fly.velocity=-3300*28/60.0;
        ShotTransit.FiringState state=ShotTransit.FiringState.WAITING;
        for(int i=0;i<40&&state==ShotTransit.FiringState.WAITING;i++){t+=.02;tick(op,t);state=assist.getFiringState();}
        check(state==ShotTransit.FiringState.REVERSING,"gate opens into brief reverse: "+state);
        close(power(hw,NameConstants.TRANSIT),-.4,"feed reverse power");
        close(servo(hw,NameConstants.BLOCKER),.55,"gate open");
        t+=.12;tick(op,t);
        check(assist.getFiringState()==ShotTransit.FiringState.FEEDING&&assist.canFire(),"feeding after reverse time");
        close(power(hw,NameConstants.TRANSIT),1,"transit feed power");
        close(power(hw,NameConstants.INTAKE),1,"intake shot feed power");
        // Feeding continues until explicitly cancelled even if speed or aim degrade.
        fly.velocity=0;od.pose=new Pose2D(DistanceUnit.INCH,100,84,AngleUnit.DEGREES,20);
        for(int i=0;i<10;i++){t+=.02;tick(op,t);}
        check(assist.getFiringState()==ShotTransit.FiringState.FEEDING,"feed continues once committed");
        check(!controls.getAutoAim().isLocked(),"aim unlocks beyond hysteresis");
        // Invalid target (Pinpoint not READY) cancels the committed feed.
        od.status=GoBildaPinpointDriver.DeviceStatus.CALIBRATING;t+=.02;tick(op,t);
        check(assist.getFiringState()==ShotTransit.FiringState.WAITING,"invalid target cancels feed");
        close(power(hw,NameConstants.TRANSIT),0,"transit stops on invalid target");
        close(servo(hw,NameConstants.BLOCKER),.38,"gate closes on invalid target");
        close(((ShooterSubsystem)field(op,"shooter")).getTargetRpm(),0,"flywheel stops on invalid target target");
        t+=.02;tick(op,t);close(fly.power,0,"flywheel stops on invalid target output");
        od.status=GoBildaPinpointDriver.DeviceStatus.READY;
        od.pose=new Pose2D(DistanceUnit.INCH,100,84,AngleUnit.DEGREES,0);fly.velocity=-3300*28/60.0;
        for(int i=0;i<40&&assist.getFiringState()!=ShotTransit.FiringState.FEEDING;i++){t+=.02;tick(op,t);}
        check(assist.getFiringState()==ShotTransit.FiringState.FEEDING,"re-acquires feed");
        // Jam (KQ LT) reverses intake+transit, stops flywheel and cancels the feed.
        op.gamepad1.left_trigger=1;t+=.02;tick(op,t);
        check(assist.getFiringState()==ShotTransit.FiringState.WAITING,"jam cancels feed");
        close(power(hw,NameConstants.TRANSIT),-1,"jam transit reverse");
        close(power(hw,NameConstants.INTAKE),-1,"jam intake reverse");
        close(servo(hw,NameConstants.BLOCKER),.38,"jam closes gate");
        close(((ShooterSubsystem)field(op,"shooter")).getTargetRpm(),0,"jam stops flywheel target");
        t+=.02;tick(op,t);close(fly.power,0,"jam stops flywheel output");
        op.gamepad1.left_trigger=0;
        for(int i=0;i<40&&assist.getFiringState()!=ShotTransit.FiringState.FEEDING;i++){t+=.02;tick(op,t);}
        check(assist.getFiringState()==ShotTransit.FiringState.FEEDING,"feeds again after jam release");
        close(servo(hw,NameConstants.TURRET_LEFT),.5,"turret stays fixed while shooting");
        // Releasing B stops everything.
        op.gamepad1.b=false;t+=.02;tick(op,t);
        allMotorsStopped(hw);
        close(servo(hw,NameConstants.BLOCKER),.38,"release closes gate");
        check(assist.getFiringState()==ShotTransit.FiringState.IDLE,"release returns to IDLE");
        // Left-stick reset cancels a held B; held B does not resume.
        op.gamepad1.b=true;t+=.02;tick(op,t);
        op.gamepad1.left_stick_button=true;t+=.02;tick(op,t);allMotorsStopped(hw);
        t+=.02;tick(op,t);close(fly.power,0,"held B must not resume after left-stick reset");
        op.gamepad1.b=false;op.gamepad1.left_stick_button=false;t+=.02;tick(op,t);
        check(op.hardwareMap.get(DcMotorEx.class,NameConstants.FRONT_LEFT).direction==DcMotor.Direction.REVERSE,
            "jrc left motor direction");
        check(op.hardwareMap.get(DcMotorEx.class,NameConstants.FRONT_RIGHT).direction==DcMotor.Direction.FORWARD,
            "jrc right motor direction");
        // A conflicting command must cancel the WHOLE composition, including the B child.
        op.gamepad1.b=true;t+=.02;tick(op,t);
        DriveSubsystem drive=(DriveSubsystem)field(op,"drive");
        Command takeover=new RunCommand(()->{},drive);
        scheduler().schedule(takeover);
        check(!scheduler().isScheduled(controls),"resource conflict cancels parent");
        allMotorsStopped(op.hardwareMap);
        close(servo(hw,NameConstants.BLOCKER),.38,"cancel closes blocker");
        check(!op.gamepad1.rumbling,"cancel stops rumble");
        scheduler().cancelAll();scheduler().reset();com.qualcomm.robotcore.util.ElapsedTime.now=0;
    }

    /** Ported Overtune AutoShootCommand: speed-gated feed, feed duration, timeout, invalid localization. */
    static void autoShoot() {
        scheduler().reset();
        for(int run=0;run<2;run++) {
            boolean ready=run==0;
            com.qualcomm.robotcore.util.ElapsedTime.now=0;
            HardwareMap hw=BehaviorReplay.hardware(true);
            Follower follower=new Follower();follower.pose=new com.pedropathing.math.Pose(100,84,0);
            ShooterSubsystem shooter=new ShooterSubsystem(hw);IntakeSubsystem intake=new IntakeSubsystem(hw);
            BlockerSubsystem blocker=new BlockerSubsystem(hw);
            boolean[] loc={false};
            AutoShootCommand cmd=new AutoShootCommand(follower,shooter,new ShotIntake(intake),
                new ShotTransit(intake,blocker),new ShotTarget(),()->loc[0],500,3000,false);
            cmd.initialize();
            check(follower.events.contains("stop")&&follower.events.contains("motorStop"),"auto shot stops follower");
            double t=0;
            t+=.02;com.qualcomm.robotcore.util.ElapsedTime.now=t;cmd.execute();
            check(cmd.getStatus().equals("Waiting for valid localization"),"waits for localization");
            close(shooter.getTargetRpm(),0,"no spin without localization");
            loc[0]=true;
            hw.get(DcMotorEx.class,NameConstants.SHOOTER_0).velocity=ready?-3300*28/60.0:0;
            int feedingTicks=0;boolean sawUnreadyClosed=false;
            while(!cmd.isFinished()&&t<10) {
                t+=.02;com.qualcomm.robotcore.util.ElapsedTime.now=t;cmd.execute();
                if(!shooter.isReady()&&cmd.getStatus().equals("Waiting for shooter speed")) {
                    close(power(hw,NameConstants.TRANSIT),0,"auto: no transit before speed");
                    close(servo(hw,NameConstants.BLOCKER),.38,"auto: gate closed before speed");
                    sawUnreadyClosed=true;
                }
                if(cmd.getStatus().startsWith("Shooting")) {
                    feedingTicks++;
                    close(power(hw,NameConstants.TRANSIT),1,"auto transit feed");
                    close(power(hw,NameConstants.INTAKE),1,"auto intake feed");
                }
            }
            check(sawUnreadyClosed,"auto waited for speed");
            cmd.end(false);
            if(ready) {
                check(!cmd.hasTimedOut()&&cmd.getStatus().equals("Shot complete"),"auto shot completes: "+cmd.getStatus());
                check(feedingTicks>=24&&feedingTicks<=26,"feed lasts 500ms: "+feedingTicks);
            } else {
                check(cmd.hasTimedOut()&&cmd.getStatus().startsWith("Shot timed out"),"auto shot times out");
                check(Math.abs(t-3.0)<.021,"timeout at 3000ms: "+t);
                check(feedingTicks==0,"no feed when never ready");
            }
            allMotorsStopped(hw);
            close(servo(hw,NameConstants.BLOCKER),.38,"auto end closes gate");
        }
        com.qualcomm.robotcore.util.ElapsedTime.now=0;scheduler().reset();
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
        DashboardTuner.motorName=new String[]{NameConstants.FRONT_LEFT,NameConstants.SHOOTER_0,NameConstants.SHOOTER_1,""};
        DashboardTuner.servoName=new String[]{"","","",""};
        DashboardTuner.colorSensorName=new String[]{"","","",""};
        DashboardTuner.servoTarget=new double[]{-1,-1,-1,-1};
        DashboardTuner.motorTarget=new double[]{.6,0,0,0};
        DashboardTuner.isPositionCloseLoop=new boolean[4];DashboardTuner.isVelocityCloseLoop=new boolean[4];
        DashboardTuner.slaveTo=new int[]{-1,-1,1,-1};
        DashboardTuner.slavePowerScale=new double[]{1,1,1,1};
        HardwareMap hw=BehaviorReplay.hardware(false);
        DashboardTuner tuner=new DashboardTuner(hw);
        tuner.applyTargets();
        close(hw.get(DcMotorEx.class,NameConstants.FRONT_LEFT).power,.6,"tuner runs");
        check(hw.get(DcMotorEx.class,NameConstants.FRONT_LEFT).direction==DcMotor.Direction.REVERSE,"tuner drive direction matches TeleOp");
        check(hw.get(DcMotorEx.class,NameConstants.SHOOTER_0).direction==DcMotor.Direction.FORWARD
            &&hw.get(DcMotorEx.class,NameConstants.SHOOTER_1).direction==DcMotor.Direction.REVERSE,"shooter directions match TeleOp");
        // shooterMotor1 velocity loop, shooterMotor2 follows with the same power.
        DashboardTuner.isVelocityCloseLoop=new boolean[]{false,true,false,false};
        DashboardTuner.kP=new double[]{0,.001,0,0};DashboardTuner.kF=new double[]{0,.0003,0,0};
        DashboardTuner.motorTarget=new double[]{0,1500,0,0};
        hw.get(DcMotorEx.class,NameConstants.SHOOTER_0).velocity=1000;
        tuner.applyTargets();
        double lead=hw.get(DcMotorEx.class,NameConstants.SHOOTER_0).power;
        check(lead>0,"leader drives positive (shooting) power");
        close(hw.get(DcMotorEx.class,NameConstants.SHOOTER_1).power,lead,"follower copies leader power");
        // Encoder-less shooterMotor2 must not run its own loop.
        DashboardTuner.slaveTo=new int[]{-1,-1,-1,-1};
        DashboardTuner.isVelocityCloseLoop=new boolean[]{false,false,true,false};
        DashboardTuner.motorTarget=new double[]{0,0,1500,0};
        tuner.applyTargets();
        close(hw.get(DcMotorEx.class,NameConstants.SHOOTER_1).power,0,"shooterMotor2 closed loop blocked");
        // Live rename rebinding.
        DashboardTuner.motorName=new String[]{NameConstants.INTAKE,"","",""};
        DashboardTuner.isVelocityCloseLoop=new boolean[4];
        DashboardTuner.motorTarget=new double[]{.4,0,0,0};
        tuner.applyTargets();
        close(hw.get(DcMotorEx.class,NameConstants.INTAKE).power,.4,"live rebind");
        close(hw.get(DcMotorEx.class,NameConstants.FRONT_LEFT).power,0,"old binding stopped");
        tuner.stop();allMotorsStopped(hw);
        DashboardTuner.motorName=new String[]{"","","",""};
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

    static void fixedTurretChassisAim() {
        scheduler().reset();
        check(TurretSubsystem.FIXED_POSITION_ENABLED,"fixed mode is enabled by default");
        HardwareMap hardware=BehaviorReplay.hardware(true);
        TurretSubsystem defaultTurret=new TurretSubsystem(hardware);
        check(defaultTurret.isFixedPositionEnabled(),"production turret is fixed");
        scheduler().reset();
        hardware=BehaviorReplay.hardware(true);
        LocalizationSubsystem localization=new LocalizationSubsystem(hardware);
        TurretSubsystem turret=new TurretSubsystem(hardware,true);
        check(turret.isFixedPositionEnabled(),"explicit test instance locked");
        close(hardware.get(Servo.class,NameConstants.TURRET_LEFT).position,.5,"fixed INIT left");
        close(hardware.get(Servo.class,NameConstants.TURRET_RIGHT).position,.5,"fixed INIT right");
        for(int i=0;i<100;i++) {
            hardware.get(IMU.class,NameConstants.IMU).yaw=i*17;
            localization.update();
            turret.toggleAimMode(true);turret.setAimMode(true,true);
            turret.toggleLockMode(i*17);turret.adjustLeft();turret.adjustRight();
            turret.reset();turret.update(localization);
            close(hardware.get(Servo.class,NameConstants.TURRET_LEFT).position,.5,"locked left");
            close(hardware.get(Servo.class,NameConstants.TURRET_RIGHT).position,.5,"locked right");
            check(!turret.isAimMode()&&!turret.isLockMode(),"all aim/heading toggles ignored");
        }
        scheduler().reset();
    }

    public static void main(String[] args)throws Exception {
        shootingAndOwnership();pathSequence();timedDrive();leafCommands();tunerStop();
        poseHandoff();fixedTurretChassisAim();autoShoot();
        System.out.println("PASS: resource arbitration, OT shoot-assist gate/feed/jam/release, AutoShoot duration/timeout, reset, pose handoff, cancellation, path handoff/hold, demo timing, tuner STOP, fixed turret");
    }
}
