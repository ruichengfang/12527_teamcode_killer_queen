import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.tests.DashTuner;
import com.acmerobotics.dashboard.FtcDashboard;
import java.lang.reflect.*;
import java.util.*;

/** Same fixture runs against original Git sources and the current working tree. */
public final class BehaviorReplay {
    static final String[] MOTORS = {NameConstants.FRONT_LEFT, NameConstants.FRONT_RIGHT,
        NameConstants.BACK_LEFT, NameConstants.BACK_RIGHT, NameConstants.INTAKE,
        NameConstants.TRANSIT, NameConstants.SHOOTER_0, NameConstants.SHOOTER_1};
    static final String[] SERVOS = {NameConstants.TURRET_LEFT, NameConstants.TURRET_RIGHT,
        NameConstants.BLOCKER};

    static HardwareMap hardware(boolean pinpoint) {
        HardwareMap map = new HardwareMap();
        for (String name : MOTORS) map.devices.put(name, new DcMotorEx());
        for (String name : SERVOS) map.devices.put(name, new Servo());
        map.devices.put(NameConstants.IMU, new IMU());
        if (pinpoint) map.devices.put(NameConstants.PINPOINT, new GoBildaPinpointDriver());
        return map;
    }

    static void snapshot(String label, OpMode op) throws Exception {
        StringBuilder s = new StringBuilder(label);
        for (Map.Entry<String,Object> e : op.hardwareMap.devices.entrySet()) {
            Object d=e.getValue(); s.append('|').append(e.getKey()).append('=');
            if (d instanceof DcMotorEx) {
                DcMotorEx m=(DcMotorEx)d;
                s.append(m.power).append(',').append(m.velocityTarget).append(',')
                 .append(m.direction).append(',').append(m.mode).append(',').append(m.zero)
                 .append(',').append(m.pid);
            } else if (d instanceof Servo) {
                Servo servo=(Servo)d; s.append(servo.position).append(',').append(servo.direction);
            } else if (d instanceof IMU) {
                IMU imu=(IMU)d; s.append(imu.yaw).append(',').append(imu.resets);
            } else if (d instanceof GoBildaPinpointDriver) {
                GoBildaPinpointDriver od=(GoBildaPinpointDriver)d;
                s.append(od.getPosition().getX(DistanceUnit.INCH)).append(',')
                 .append(od.getPosition().getY(DistanceUnit.INCH)).append(',')
                 .append(od.getHeading(AngleUnit.DEGREES)).append(',').append(od.resets)
                 .append(',').append(od.ox).append(',').append(od.oy);
            }
        }
        if (op instanceof Teleop) {
            for (String field : new String[]{"localization","drive","intake","shooter","turret","blocker"}) {
                Field f=Teleop.class.getDeclaredField(field); f.setAccessible(true);
                Object subsystem=f.get(op);
                Method[] methods=subsystem.getClass().getDeclaredMethods();
                Arrays.sort(methods,Comparator.comparing(Method::getName));
                for (Method m : methods) {
                    if (Modifier.isPublic(m.getModifiers()) && m.getParameterCount()==0
                            && m.getName().matches("(get|is|has).*")
                            && !m.getName().equals("isFixedPositionEnabled")) {
                        s.append('|').append(field).append('.').append(m.getName()).append('=')
                         .append(m.invoke(subsystem));
                    }
                }
            }
            s.append('|').append(op.telemetry.values).append('|').append(op.gamepad1.rumbling);
        }
        System.out.println(s);
    }

    static void tick(Teleop op, double time, String label) throws Exception {
        op.clock=time; op.loop(); snapshot(label,op);
    }

    static void scripted(boolean pinpoint) throws Exception {
        AutoEndPose.clear();
        Teleop op=new Teleop(); op.hardwareMap=hardware(pinpoint); op.init();
        Gamepad g=op.gamepad1;
        // B/left-stick/up held before START: no accidental firing/reset/increment.
        g.b=g.start=g.left_stick_button=g.dpad_up=true; op.start();
        tick(op,0,"heldBeforeStart-"+pinpoint);
        g.b=g.start=g.left_stick_button=g.dpad_up=false; tick(op,.1,"releaseHeldB");
        // All manual buttons must be suppressed by newly pressed B, except driving.
        g.a=g.b=g.x=g.y=g.dpad_left=g.dpad_right=g.left_bumper=g.right_bumper=true;
        g.left_stick_y=-1; g.left_stick_x=1; g.right_stick_x=.7f;
        tick(op,1,"pressB"); tick(op,1.099,"beforeFeed"); tick(op,1.1,"feedBoundary");
        // Start is no longer reset; left-stick reset cancels firing, including not-ready Pinpoint.
        g.start=true;
        if(pinpoint) op.hardwareMap.get(GoBildaPinpointDriver.class,NameConstants.PINPOINT).status=
                GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        tick(op,2.1,"startDoesNotReset");
        g.left_stick_button=true;tick(op,2.11,"leftStickCancelsFeed");
        tick(op,2.12,"heldBDoesNotRestartAfterReset");
        g.b=false; g.dpad_up=true;
        tick(op,2.2,"releaseBwhileLBAndUpHeld");
        tick(op,2.3,"heldLBAndUp");
        g.left_bumper=g.dpad_up=g.a=g.x=g.y=g.start=g.left_stick_button=g.dpad_left=g.dpad_right=false;
        tick(op,2.4,"releaseButtons");
        g.left_bumper=g.dpad_down=true; g.right_bumper=false; g.right_trigger=.6f;g.left_trigger=.8f;
        tick(op,2.5,"manualLBDownRTwins");
        g.dpad_up=true; tick(op,2.6,"oppositeDpad");
        g.right_trigger=0; tick(op,2.7,"reverse");
        g.b=true; tick(op,3,"secondShot"); tick(op,4,"secondShotFeed");
        op.stop(); snapshot("stopWhileFiring",op);
        CommandScheduler.getInstance().reset();
    }

    static void randomized(boolean pinpoint) throws Exception {
        AutoEndPose.clear();
        Teleop op=new Teleop(); op.hardwareMap=hardware(pinpoint); op.init(); op.start();
        Random random=new Random(12527);
        Field[] fields=Gamepad.class.getFields(); Arrays.sort(fields,Comparator.comparing(Field::getName));
        double time=0;
        for(int i=0;i<3000;i++) {
            time+=i%67==0?0:(i%53==0?1.5:.02);
            for(Field f:fields) {
                if(f.getName().equals("rumbling"))continue;
                if(f.getType()==boolean.class && random.nextDouble()<.075)
                    f.setBoolean(op.gamepad1,!f.getBoolean(op.gamepad1));
                else if(f.getType()==float.class) {
                    float value=(float)(random.nextDouble()*2-1);
                    if(f.getName().endsWith("trigger"))value=Math.abs(value);
                    if(i%23==0)value=.05f;
                    f.setFloat(op.gamepad1,value);
                }
            }
            op.hardwareMap.get(IMU.class,NameConstants.IMU).yaw=random.nextDouble()*720-360;
            op.hardwareMap.get(DcMotorEx.class,NameConstants.SHOOTER_0).velocity=-random.nextDouble()*1800;
            if(pinpoint) {
                GoBildaPinpointDriver od=op.hardwareMap.get(GoBildaPinpointDriver.class,NameConstants.PINPOINT);
                od.status=GoBildaPinpointDriver.DeviceStatus.values()[i%3];
                od.pose=new Pose2D(DistanceUnit.INCH,random.nextDouble()*144,
                    random.nextDouble()*144,AngleUnit.DEGREES,random.nextDouble()*360);
            }
            tick(op,time,"random-"+pinpoint+"-"+i);
        }
        op.stop(); snapshot("randomStop-"+pinpoint,op);
    }

    static void savedPoseHandoff() throws Exception {
        AutoEndPose.clear(); AutoEndPose.save(24,48,Math.toRadians(135));
        Teleop op=new Teleop();op.hardwareMap=hardware(true);op.init();
        GoBildaPinpointDriver od=op.hardwareMap.get(GoBildaPinpointDriver.class,NameConstants.PINPOINT);
        od.status=GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        op.init_loop();snapshot("savedPosePending",op);
        od.status=GoBildaPinpointDriver.DeviceStatus.READY;
        op.init_loop();snapshot("savedPoseRestored",op);
        op.start();op.gamepad1.left_stick_y=-1;tick(op,.02,"driveUsingRestoredYaw");
        od.status=GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        op.gamepad1.left_stick_button=true;tick(op,.04,"resetPendingReady");
        od.status=GoBildaPinpointDriver.DeviceStatus.READY;
        tick(op,.06,"resetToDefaultWhenReady");
        op.stop();snapshot("handoffStop",op);AutoEndPose.clear();
    }

    static void tuner() throws Exception {
        DashTuner op=new DashTuner();op.hardwareMap=hardware(true);
        DashTuner.motorName=new String[]{MOTORS[0],MOTORS[1],MOTORS[2],MOTORS[3]};
        DashTuner.servoName=new String[]{SERVOS[0],SERVOS[1],SERVOS[2],""};
        DashTuner.colorSensorName=new String[]{"color","","",""};
        class Sensor implements ColorSensor,DistanceSensor {
            public int red(){return 3;} public int green(){return 4;} public int blue(){return 5;}
            public int alpha(){return 6;} public double getDistance(DistanceUnit unit){return 17.5;}
        }
        op.hardwareMap.devices.put("color",new Sensor());
        DashTuner.isVelocityCloseLoop=new boolean[]{true,true,false,false};
        DashTuner.isPositionCloseLoop=new boolean[]{false,false,true,false};
        DashTuner.slaveTo=new int[]{-1,0,-1,-1};
        DashTuner.kP=new double[]{.1,.2,.3,.4};DashTuner.kF=new double[]{.01,.02,.03,.04};
        op.init();op.start();
        for(int i=0;i<20;i++) {
            DashTuner.motorTarget=new double[]{100+i,200+i,10+i,.7};
            DashTuner.servoTarget=new double[]{.2+i*.01,.5,.8,0};
            for(int j=0;j<4;j++) {
                DcMotorEx m=op.hardwareMap.get(DcMotorEx.class,MOTORS[j]);
                m.velocity=i*10+j;m.position=i+j;
            }
            FtcDashboard.getInstance().packets.clear();op.loop();snapshot("tuner-"+i,op);
            System.out.println(FtcDashboard.getInstance().packets);
        }
        op.stop(); // Current version explicitly zeroes motors; old SDK did this outside OpMode.
    }

    public static void main(String[] args) throws Exception {
        Locale.setDefault(Locale.ROOT);
        scripted(true);scripted(false);randomized(true);randomized(false);savedPoseHandoff();
        Teleop unstarted=new Teleop();unstarted.hardwareMap=hardware(true);unstarted.init();
        unstarted.stop();snapshot("stopDuringInit",unstarted);
        tuner();
    }
}
