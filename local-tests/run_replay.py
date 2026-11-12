#!/usr/bin/env python3
"""Replay original and refactored Java against the SAME deterministic hardware boundary.

Requires Python 3, a JDK, and the project's actual FTCLib 2.1.1 classes.jar.
Usage: python3 local-tests/run_replay.py --ftclib /path/to/classes.jar
No robot/network access. Original files are read from Git, never checked out over work.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
JAVA_ROOT = Path('TeamCode/src/main/java/org/firstinspires/ftc/teamcode')
BASELINE = '96796900f20abc62661e565ae0a93e830493b372'

# Only FTC/Android/Pedro hardware boundaries are substituted. Production commands,
# all six original mechanism subsystems, PID math and the real FTCLib scheduler run.
STUBS = {
'com/qualcomm/robotcore/hardware/HardwareMap.java': '''
package com.qualcomm.robotcore.hardware;
import java.util.*;
public class HardwareMap {
 public final Map<String,Object> devices = new TreeMap<>();
 public <T> T get(Class<T> type,String name) {
  if (!devices.containsKey(name)) throw new IllegalArgumentException(name);
  return type.cast(devices.get(name));
 }
 public <T> T tryGet(Class<T> type,String name) {
  Object d=devices.get(name); return type.isInstance(d)?type.cast(d):null;
 }
}''',
'com/qualcomm/robotcore/hardware/DcMotorSimple.java': '''
package com.qualcomm.robotcore.hardware;
public class DcMotorSimple {
 public enum Direction {FORWARD,REVERSE}
}''',
'com/qualcomm/robotcore/hardware/DcMotor.java': '''
package com.qualcomm.robotcore.hardware;
public class DcMotor extends DcMotorSimple {
 public enum RunMode {RUN_WITHOUT_ENCODER,RUN_USING_ENCODER}
 public enum ZeroPowerBehavior {BRAKE,FLOAT}
 public double power; public Direction direction; public RunMode mode;
 public ZeroPowerBehavior zero;
 public void setPower(double v){power=v;} public double getPower(){return power;}
 public void setDirection(Direction v){direction=v;} public void setMode(RunMode v){mode=v;}
 public RunMode getMode(){return mode;}
 public void setZeroPowerBehavior(ZeroPowerBehavior v){zero=v;}
}''',
'com/qualcomm/robotcore/hardware/DcMotorEx.java': '''
package com.qualcomm.robotcore.hardware;
public class DcMotorEx extends DcMotor {
 public double velocity,velocityTarget; public int position; public PIDFCoefficients pid;
 public double getVelocity(){return velocity;} public void setVelocity(double v){velocityTarget=v;}
 public int getCurrentPosition(){return position;}
 public void setPIDFCoefficients(RunMode mode,PIDFCoefficients p){pid=p;}
}''',
'com/qualcomm/robotcore/hardware/PIDFCoefficients.java': '''
package com.qualcomm.robotcore.hardware;
public class PIDFCoefficients {
 public final double p,i,d,f;
 public PIDFCoefficients(double p,double i,double d,double f){this.p=p;this.i=i;this.d=d;this.f=f;}
 public String toString(){return p+","+i+","+d+","+f;}
}''',
'com/qualcomm/robotcore/hardware/Servo.java': '''
package com.qualcomm.robotcore.hardware;
public class Servo {
 public enum Direction {FORWARD,REVERSE}
 public Direction direction; public double position;
 public void setDirection(Direction d){direction=d;}
 public void setPosition(double p){position=Math.max(0,Math.min(1,p));}
 public double getPosition(){return position;}
}''',
'com/qualcomm/robotcore/hardware/Gamepad.java': '''
package com.qualcomm.robotcore.hardware;
public class Gamepad {
 public boolean a,b,x,y,dpad_up,dpad_down,dpad_left,dpad_right,start,left_stick_button,left_bumper,right_bumper;
 public float left_stick_x,left_stick_y,right_stick_x,right_trigger,left_trigger;
 public boolean rumbling;
 public void rumble(double l,double r,int ms){rumbling=true;}
 public void stopRumble(){rumbling=false;}
}''',
'com/qualcomm/robotcore/hardware/IMU.java': '''
package com.qualcomm.robotcore.hardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
public class IMU {
 public static class Parameters {public Parameters(Object orientation){}}
 public double yaw; public int resets;
 public void initialize(Parameters p){} public void resetYaw(){yaw=0;resets++;}
 public Angles getRobotYawPitchRollAngles(){return new Angles(yaw);}
 public static class Angles {
  private final double yaw; Angles(double y){yaw=y;}
  public double getYaw(AngleUnit u){return u==AngleUnit.RADIANS?Math.toRadians(yaw):yaw;}
 }
}''',
'com/qualcomm/hardware/rev/RevHubOrientationOnRobot.java': '''
package com.qualcomm.hardware.rev;
public class RevHubOrientationOnRobot {
 public enum LogoFacingDirection {LEFT,RIGHT} public enum UsbFacingDirection {UP}
 public static LogoFacingDirection lastLogo; public static UsbFacingDirection lastUsb;
 public RevHubOrientationOnRobot(LogoFacingDirection l,UsbFacingDirection u){lastLogo=l;lastUsb=u;}
}''',
'org/firstinspires/ftc/robotcore/external/navigation/AngleUnit.java': '''
package org.firstinspires.ftc.robotcore.external.navigation;
public enum AngleUnit {RADIANS,DEGREES}''',
'org/firstinspires/ftc/robotcore/external/navigation/DistanceUnit.java': '''
package org.firstinspires.ftc.robotcore.external.navigation;
public enum DistanceUnit {MM,INCH,CM;
 public double toCm(double v){return this==INCH?v*2.54:this==MM?v/10:v;}}''',
'org/firstinspires/ftc/robotcore/external/navigation/Pose2D.java': '''
package org.firstinspires.ftc.robotcore.external.navigation;
public class Pose2D {
 private final double x,y,h;
 public Pose2D(DistanceUnit u,double x,double y,AngleUnit a,double h){this.x=x;this.y=y;this.h=a==AngleUnit.RADIANS?Math.toDegrees(h):h;}
 public double getX(DistanceUnit u){return x;} public double getY(DistanceUnit u){return y;}
 public double getHeading(AngleUnit u){return u==AngleUnit.DEGREES?h:Math.toRadians(h);}
}''',
'com/qualcomm/hardware/gobilda/GoBildaPinpointDriver.java': '''
package com.qualcomm.hardware.gobilda;
import org.firstinspires.ftc.robotcore.external.navigation.*;
public class GoBildaPinpointDriver {
 public enum GoBildaOdometryPods {goBILDA_4_BAR_POD} public enum EncoderDirection {FORWARD}
 public enum DeviceStatus {READY,CALIBRATING,FAULT_NO_PODS_DETECTED}
 public DeviceStatus status=DeviceStatus.READY;
 public Pose2D pose=new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,0);
 public int resets; public double ox,oy;
 public void setOffsets(double x,double y,DistanceUnit u){ox=x;oy=y;}
 public void setEncoderResolution(GoBildaOdometryPods p){}
 public void setEncoderDirections(EncoderDirection x,EncoderDirection y){}
 public void resetPosAndIMU(){resets++;} public void update(){}
 public DeviceStatus getDeviceStatus(){return status;}
 public Pose2D getPosition(){return pose;}
 public void setPosition(Pose2D p){pose=p;}
 public double getHeading(AngleUnit u){return pose.getHeading(u);}
 public double velX,velY;
 public double getVelX(DistanceUnit u){return velX;} public double getVelY(DistanceUnit u){return velY;}
}''',
'com/qualcomm/robotcore/eventloop/opmode/TeleOp.java': '''
package com.qualcomm.robotcore.eventloop.opmode;
public @interface TeleOp {String name();String group() default "";}''',
'com/qualcomm/robotcore/eventloop/opmode/OpMode.java': '''
package com.qualcomm.robotcore.eventloop.opmode;
import com.qualcomm.robotcore.hardware.*;
public abstract class OpMode {
 public HardwareMap hardwareMap=new HardwareMap(); public Gamepad gamepad1=new Gamepad();
 public Telemetry telemetry=new Telemetry(); public double clock;
 public double getRuntime(){return clock;}
 public abstract void init(); public void init_loop(){} public abstract void loop(); public void start(){} public void stop(){}
 public static class Telemetry {
  public final java.util.Map<String,String> values=new java.util.TreeMap<>();
  public void addLine(String s){}
  public void addData(String k,Object v){values.put(k,String.valueOf(v));}
  public void addData(String k,String f,Object... v){values.put(k,String.format(java.util.Locale.ROOT,f,v));}
  public void update(){}
 }
}''',
'com/pedropathing/paths/Path.java': '''
package com.pedropathing.paths;
public class Path {
 public final String id; public com.pedropathing.math.Pose end;
 public Path(String id){this.id=id;}
 public Path(com.pedropathing.math.Pose a,com.pedropathing.math.Pose b){id="path";end=b;}
 public Path linear(com.pedropathing.math.Pose a,com.pedropathing.math.Pose b){return this;}
 public Path constant(com.pedropathing.math.Pose p){return this;}
}''',
'com/pedropathing/follower/Follower.java': '''
package com.pedropathing.follower;
import com.pedropathing.paths.Path;
public class Follower {
 public final java.util.List<String> events=new java.util.ArrayList<>();
 public final Drive drivetrain=new Drive(); public boolean atEnd; public int updates;
 public boolean autoAdvance; public Path current;
 public com.pedropathing.math.Pose pose=new com.pedropathing.math.Pose(0,0,0);
 public void setPose(com.pedropathing.math.Pose p){pose=p;}
 public com.pedropathing.math.Pose pose(){return pose;}
 public com.pedropathing.math.Velocity velocity=new com.pedropathing.math.Velocity(0,0,0);
 public com.pedropathing.math.Velocity velocity(){return velocity;}
 public Path currentPath(){return current;} public double distanceToEndpoint(){return 0;}
 public int pathIndex(){return 0;}
 public void update(){updates++;if(autoAdvance&&current!=null){pose=current.end;atEnd=true;}}
 public void follow(Path p){events.add("follow:"+p.id);current=p;atEnd=false;}
 public boolean atParametricEnd(){return atEnd;}
 public void stop(){events.add("stop");}
 public class Drive {public void stop(){events.add("motorStop");}}
}''',
'com/qualcomm/robotcore/hardware/ColorSensor.java': '''
package com.qualcomm.robotcore.hardware;
public interface ColorSensor {int red();int green();int blue();int alpha();}''',
'com/qualcomm/robotcore/hardware/DistanceSensor.java': '''
package com.qualcomm.robotcore.hardware;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public interface DistanceSensor {double getDistance(DistanceUnit u);}''',
'android/graphics/Color.java': '''
package android.graphics;
public class Color {public static void RGBToHSV(int r,int g,int b,float[] hsv){hsv[0]=r;hsv[1]=g;hsv[2]=b;}}''',
'com/acmerobotics/dashboard/config/Config.java': '''
package com.acmerobotics.dashboard.config;
public @interface Config {}''',
'com/acmerobotics/dashboard/telemetry/TelemetryPacket.java': '''
package com.acmerobotics.dashboard.telemetry;
public class TelemetryPacket {
 public final java.util.Map<String,Object> values=new java.util.TreeMap<>();
 public void put(String k,Object v){values.put(k,v);}
}''',
'com/acmerobotics/dashboard/FtcDashboard.java': '''
package com.acmerobotics.dashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
public class FtcDashboard {
 private static final FtcDashboard INSTANCE=new FtcDashboard();
 public final java.util.List<String> packets=new java.util.ArrayList<>();
 public static FtcDashboard getInstance(){return INSTANCE;}
 public void sendTelemetryPacket(TelemetryPacket p){packets.add(p.values.toString());}
}''',
}

STUBS.update({
'com/qualcomm/robotcore/util/ElapsedTime.java': '''
package com.qualcomm.robotcore.util;
public class ElapsedTime {
 public static double now;
 private double start=now;
 public void reset(){start=now;}
 public double seconds(){return now-start;}
 public double milliseconds(){return seconds()*1000;}
}''',
'com/qualcomm/robotcore/eventloop/opmode/Autonomous.java': '''
package com.qualcomm.robotcore.eventloop.opmode;
public @interface Autonomous {String name();String group() default "";}''',
'com/qualcomm/robotcore/eventloop/opmode/LinearOpMode.java': '''
package com.qualcomm.robotcore.eventloop.opmode;
public abstract class LinearOpMode extends OpMode {
 public boolean abortBeforeStart,started,stopped;public int initChecks,activeTicks=8;
 public abstract void runOpMode();public void init(){}public void loop(){}
 public void waitForStart(){started=!abortBeforeStart;}
 public boolean isStarted(){if(!abortBeforeStart&&++initChecks>=3)started=true;return started;}
 public boolean isStopRequested(){return stopped||abortBeforeStart;}
 public boolean opModeIsActive(){if(activeTicks--<=0){stopped=true;return false;}clock+=.02;com.qualcomm.robotcore.util.ElapsedTime.now+=.02;return true;}
 public void sleep(long ms){clock+=ms/1000.0;}public void idle(){}
}''',
'com/pedropathing/math/Pose.java': '''
package com.pedropathing.math;
public class Pose {
 private final double x,y,h;public Pose(double x,double y,double h){this.x=x;this.y=y;this.h=h;}
 public double x(){return x;}public double y(){return y;}public double heading(){return h;}
}''',
'com/pedropathing/math/Velocity.java': '''
package com.pedropathing.math;
public class Velocity {
 public final double vx,vy,omega;public Velocity(double x,double y,double w){vx=x;vy=y;omega=w;}
 public double linSpeed(){return Math.hypot(vx,vy);}
}''',
'com/pedropathing/api/PoseFactory.java': '''
package com.pedropathing.api;
import com.pedropathing.math.Pose;
public class PoseFactory {public static PoseFactory degrees(){return new PoseFactory();}
 public Pose of(double x,double y,double degrees){return new Pose(x,y,Math.toRadians(degrees));}}
''',
'com/pedropathing/api/Paths.java': '''
package com.pedropathing.api;
import com.pedropathing.math.Pose;import com.pedropathing.paths.Path;
public class Paths {
 public static Path line(Pose a,Pose b){return new Path(a,b);}
 public static Path curve(Pose... points){return new Path(points[0],points[points.length-1]);}
}''',
'org/firstinspires/ftc/teamcode/pedro/Constants.java': '''
package org.firstinspires.ftc.teamcode.pedro;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Constants {
 public static Follower last;
 public static boolean autoAdvance=true;
 public static Follower create(HardwareMap h){last=new Follower();last.autoAdvance=autoAdvance;return last;}
}''',
})


def run(cmd, **kwargs):
    return subprocess.run(cmd, check=True, text=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ftclib', required=True, type=Path)
    parser.add_argument('--current-only', action='store_true', help='Run current lifecycle checks after intentional behavior changes')
    args = parser.parse_args()
    jar = str(args.ftclib.resolve())
    java_home = Path(os.environ.get('JAVA_HOME', '/usr'))
    javac = str(java_home/'bin/javac') if (java_home/'bin/javac').exists() else shutil.which('javac')
    java = str(java_home/'bin/java') if (java_home/'bin/java').exists() else shutil.which('java')
    with tempfile.TemporaryDirectory(prefix='killerqueen-replay-') as temp:
        root = Path(temp)
        output = {}
        for version in (('current',) if args.current_only else ('baseline', 'current')):
            source = root/version/'src'
            classes = root/version/'classes'
            source.mkdir(parents=True)
            classes.mkdir()
            for name, content in STUBS.items():
                f = source/name
                f.parent.mkdir(parents=True, exist_ok=True)
                f.write_text(content)
            if version == 'baseline':
                files = run(['git', 'ls-tree', '-r', '--name-only', BASELINE, str(JAVA_ROOT)],
                            cwd=ROOT, capture_output=True).stdout.splitlines()
            else:
                files = [str(f.relative_to(ROOT)) for f in (ROOT/JAVA_ROOT).rglob('*.java')]
            for name in files:
                rel = Path(name).relative_to(JAVA_ROOT)
                if rel.parts[0] == 'pedro' or (version == 'baseline' and rel.parts[0] == 'autopath'):
                    continue
                # DashTuner was intentionally replaced by Spinup's DashboardTuner (not equivalence-tested).
                if str(rel) in ('tests/DashTuner.java', 'tests/ShooterAutoTune.java'):
                    continue
                content = (run(['git', 'show', f'{BASELINE}:{name}'], cwd=ROOT,
                               capture_output=True).stdout if version == 'baseline'
                           else (ROOT/name).read_text())
                f = source/'org/firstinspires/ftc/teamcode'/rel
                f.parent.mkdir(parents=True, exist_ok=True)
                f.write_text(content)
            if version == 'current':
                # Historical lifecycle checks exercise archived commands explicitly.
                # Restore their Java suffix only in the temporary test source tree;
                # the Android application does not compile these files.
                for archived in (ROOT/JAVA_ROOT/'command').glob('*.java.disabled'):
                    target = source/'org/firstinspires/ftc/teamcode/command'/archived.name[:-len('.disabled')]
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy(archived, target)
            shutil.copy(ROOT/'local-tests/BehaviorReplay.java', source)
            if version == 'current':
                shutil.copy(ROOT/'local-tests/LifecycleCheck.java', source)
                shutil.copy(ROOT/'local-tests/AutoLifecycleCheck.java', source)
            sources = [str(f) for f in source.rglob('*.java')]
            run([javac, '-encoding', 'UTF-8', '-cp', jar, '-d', str(classes), *sources],
                capture_output=False)
            output[version] = run([java, '-cp', str(classes)+os.pathsep+jar, 'BehaviorReplay'],
                                  capture_output=True).stdout.splitlines()
            if version == 'current':
                run([java, '-cp', str(classes)+os.pathsep+jar, 'LifecycleCheck'])
                run([java, '-cp', str(classes)+os.pathsep+jar, 'AutoLifecycleCheck'])
        if args.current_only:
            return
        old, new = output['baseline'], output['current']
        if old != new:
            from itertools import zip_longest
            for i, (a, b) in enumerate(zip_longest(old, new)):
                if a != b:
                    raise AssertionError(f'Replay mismatch line {i+1}\nold: {a}\nnew: {b}')
        print(f'PASS: {len(old)} identical output/state snapshots vs {BASELINE[:8]}')


if __name__ == '__main__':
    main()
