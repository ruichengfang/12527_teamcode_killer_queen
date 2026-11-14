#!/usr/bin/env python3
"""Exercise the real localization reset with deterministic Pinpoint substitutes."""
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
JAVA = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
all_stubs = runpy.run_path(str(ROOT / 'local-tests/run_replay.py'))['STUBS']
needed = (
    'com/qualcomm/robotcore/hardware/HardwareMap.java',
    'com/qualcomm/robotcore/hardware/IMU.java',
    'com/qualcomm/robotcore/hardware/DcMotorSimple.java',
    'com/qualcomm/robotcore/hardware/DcMotor.java',
    'com/qualcomm/robotcore/hardware/DcMotorEx.java',
    'com/qualcomm/robotcore/hardware/PIDFCoefficients.java',
    'com/qualcomm/hardware/rev/RevHubOrientationOnRobot.java',
    'com/qualcomm/hardware/gobilda/GoBildaPinpointDriver.java',
    'org/firstinspires/ftc/robotcore/external/navigation/AngleUnit.java',
    'org/firstinspires/ftc/robotcore/external/navigation/DistanceUnit.java',
    'org/firstinspires/ftc/robotcore/external/navigation/Pose2D.java',
)
stubs = {name: all_stubs[name] for name in needed}
stubs['com/arcrobotics/ftclib/command/SubsystemBase.java'] = (
    'package com.arcrobotics.ftclib.command; public class SubsystemBase {}')
pinpoint = 'com/qualcomm/hardware/gobilda/GoBildaPinpointDriver.java'
stubs[pinpoint] = (stubs[pinpoint]
    .replace('FAULT_NO_PODS_DETECTED}', 'FAULT_NO_PODS_DETECTED,FAULT_BAD_READ}')
    .replace('public int resets; public double ox,oy;', 'public int resets, writes, configs; public double ox,oy;')
    .replace('{ox=x;oy=y;}', '{configs++;ox=x;oy=y;}')
    .replace('setEncoderResolution(GoBildaOdometryPods p){}', 'setEncoderResolution(GoBildaOdometryPods p){configs++;}')
    .replace('setEncoderDirections(EncoderDirection x,EncoderDirection y){}', 'setEncoderDirections(EncoderDirection x,EncoderDirection y){configs++;}')
    .replace('public void resetPosAndIMU(){resets++;} public void update(){}',
             'public boolean failAfterSet, armed; public void resetPosAndIMU(){resets++;} '
             'public void update(){if(armed){status=DeviceStatus.FAULT_BAD_READ;armed=false;}}')
    .replace('public void setPosition(Pose2D p){pose=p;}',
             'public void setPosition(Pose2D p){writes++;pose=p;if(failAfterSet){armed=true;failAfterSet=false;}}'))
stubs['PinpointResetCheck.java'] = '''
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.command.PinpointTrimButtons;
public class PinpointResetCheck {
  static void check(boolean ok, String label) { if(!ok) throw new AssertionError(label); }
  static void near(double a,double b,String label) { check(Math.abs(a-b)<1e-8,label+": "+a); }
  public static void main(String[] args) {
    HardwareMap hw=new HardwareMap();
    IMU imu=new IMU(); GoBildaPinpointDriver od=new GoBildaPinpointDriver();
    hw.devices.put(NameConstants.IMU,imu); hw.devices.put(NameConstants.PINPOINT,od);
    od.pose=new Pose2D(DistanceUnit.INCH,32,44,AngleUnit.DEGREES,135);
    od.status=GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
    AutoEndPose.save(1,2,0);
    LocalizationSubsystem loc=new LocalizationSubsystem(hw);
    check(od.resets==0 && od.configs==0 && od.writes==0,"Solo INIT must not initialize or write Pinpoint");
    check(RevHubOrientationOnRobot.lastLogo == RevHubOrientationOnRobot.LogoFacingDirection.RIGHT
        && RevHubOrientationOnRobot.lastUsb == RevHubOrientationOnRobot.UsbFacingDirection.UP,
        "IMU logo right, USB up");
    loc.update();
    check(!loc.isPositionInitialized(),"wait for ready before accepting inherited pose");
    od.status=GoBildaPinpointDriver.DeviceStatus.READY;
    loc.update();
    near(loc.getXIn(),32,"inherit current device x, not saved auto pose");
    near(loc.getYIn(),44,"inherit current device y");
    near(loc.getHeadingDeg(),135,"inherit current device heading");
    check(loc.isFieldPoseValid() && od.writes==0 && od.resets==0,"inherit without writes or reset");
    loc.resetDriverHeading();
    od.pose=new Pose2D(DistanceUnit.INCH,10,20,AngleUnit.DEGREES,30);
    imu.yaw=30;
    loc.update();
    near(Math.toDegrees(loc.getManualDriveHeadingRadians()),30,"initial driver heading");
    od.failAfterSet=true;
    check(loc.resetToFieldCenter(),"accepted reset");
    check(!loc.isFieldPoseValid(),"failed read leaves pose invalid");
    od.status=GoBildaPinpointDriver.DeviceStatus.READY;
    loc.update();
    check(loc.isFieldPoseValid(),"retry restores valid pose");
    near(loc.getXIn(),72,"reset x"); near(loc.getYIn(),72,"reset y");
    near(loc.getHeadingDeg(),180,"reset heading");
    near(Math.toDegrees(loc.getManualDriveHeadingRadians()),30,"preserves driver heading on retry");
    od.pose=new Pose2D(DistanceUnit.INCH,30,110,AngleUnit.DEGREES,45);
    loc.update();
    check(loc.resetToNearestCorner(),"nearest corner accepted");
    near(loc.getXIn(),7.5,"nearest corner low x");
    near(loc.getYIn(),136.5,"nearest corner high y");
    near(loc.getHeadingDeg(),45,"nearest corner preserves field heading");
    near(Math.toDegrees(loc.getManualDriveHeadingRadians()),30,"nearest corner preserves driver heading");
    check(loc.resetToFieldCenter(),"return to center for trim checks");
    loc.update();
    near(Math.toDegrees(loc.getManualDriveHeadingRadians()),30,"no repeated reference shift");
    PinpointTrimButtons buttons=new PinpointTrimButtons();
    buttons.update(true,false,false,true);
    check(buttons.getXDirection()==1 && buttons.getYDirection()==1,"up and right edges");
    check(loc.adjustFieldPosition(buttons.getXDirection(),buttons.getYDirection()),"trim accepted");
    near(loc.getXIn(),73,"trim x"); near(loc.getYIn(),73,"trim y");
    near(loc.getHeadingDeg(),180,"trim preserves field heading");
    near(Math.toDegrees(loc.getManualDriveHeadingRadians()),30,"trim preserves driver heading");
    buttons.update(true,false,false,true);
    check(buttons.getXDirection()==0 && buttons.getYDirection()==0,"held keys do not repeat");
    buttons.update(false,false,false,false);
    buttons.update(false,true,true,false);
    check(buttons.getXDirection()==-1 && buttons.getYDirection()==-1,"down and left edges");
    check(loc.adjustFieldPosition(buttons.getXDirection(),buttons.getYDirection()),"negative trim accepted");
    near(loc.getXIn(),72,"negative x"); near(loc.getYIn(),72,"negative y");
    buttons.update(false,false,false,false);
    buttons.update(true,true,true,true);
    check(buttons.getXDirection()==0 && buttons.getYDirection()==0,"opposite edges cancel");
    od.status=GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ; loc.update();
    near(Math.toDegrees(loc.getManualDriveHeadingRadians()),30,"Pinpoint fault does not alter IMU driving");
    check(!loc.adjustFieldPosition(1,0),"reject trim without valid pose");
    od.status=GoBildaPinpointDriver.DeviceStatus.READY; loc.update();
    near(loc.getXIn(),72,"invalid trim did not change x");
    imu.yaw=Double.NaN; loc.resetDriverHeading();
    check(Double.isNaN(loc.getManualDriveHeadingRadians()),"invalid IMU is not a valid heading");
    imu.yaw=30;
    near(Math.toDegrees(loc.getManualDriveHeadingRadians()),30,"invalid reset preserves reference");
    loc.resetDriverHeading();
    near(Math.toDegrees(loc.getManualDriveHeadingRadians()),0,"driver resets IMU forward");
    imu.yaw=120;
    near(Math.toDegrees(loc.getManualDriveHeadingRadians()),90,"IMU rotation drives heading");
    for(String name:new String[]{NameConstants.FRONT_LEFT,NameConstants.FRONT_RIGHT,
        NameConstants.BACK_LEFT,NameConstants.BACK_RIGHT}) hw.devices.put(name,new DcMotorEx());
    DriveSubsystem drive=new DriveSubsystem(hw);
    drive.moveFieldCentric(1,0,0,loc.getManualDriveHeadingRadians());
    near(drive.getFrontLeftPower(),1,"90 degree heading turns forward into strafe");
    near(drive.getFrontRightPower(),-1,"90 degree heading opposite right wheel");
    drive.moveFieldCentric(1,0,0,Double.NaN);
    near(drive.getFrontLeftPower(),0,"invalid heading stops left wheel");
    near(drive.getFrontRightPower(),0,"invalid heading stops right wheel");
    HardwareMap missing=new HardwareMap(); missing.devices.put(NameConstants.IMU,new IMU());
    LocalizationSubsystem noOd=new LocalizationSubsystem(missing);
    noOd.resetDriverHeading(); missing.get(IMU.class,NameConstants.IMU).yaw=45;
    near(Math.toDegrees(noOd.getManualDriveHeadingRadians()),45,"IMU driving without Pinpoint");
    check(!noOd.resetToFieldCenter(),"missing Pinpoint rejected");
    check(!noOd.resetToNearestCorner(),"missing Pinpoint rejects corner reset");
    HardwareMap autoHw=new HardwareMap(); GoBildaPinpointDriver autoOd=new GoBildaPinpointDriver();
    autoHw.devices.put(NameConstants.IMU,new IMU()); autoHw.devices.put(NameConstants.PINPOINT,autoOd);
    LocalizationSubsystem autoLoc=new LocalizationSubsystem(autoHw,1,2,Math.toRadians(90));
    check(autoOd.resets==1 && autoOd.configs==3,"explicit auto initialization retained");
    autoLoc.update();
    near(autoLoc.getXIn(),1,"explicit auto x"); near(autoLoc.getYIn(),2,"explicit auto y");
    near(autoLoc.getHeadingDeg(),90,"explicit auto heading");
    check(autoOd.writes==1,"explicit auto writes its pose");
    System.out.println("PASS: IMU driving, Pinpoint faults, reset, trim and invalid heading stop");
  }
}'''
with tempfile.TemporaryDirectory(prefix='pinpoint-reset-') as temp:
    root = Path(temp)
    for name, content in stubs.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')
    for name in ('NameConstants.java', 'AutoEndPose.java', 'subsystem/LocalizationSubsystem.java',
                 'subsystem/DriveSubsystem.java',
                 'command/PinpointTrimButtons.java'):
        target = root / 'org/firstinspires/ftc/teamcode' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(JAVA / name, target)
    java_home = Path(os.environ.get('JAVA_HOME', ''))
    javac = str(java_home / 'bin/javac') if (java_home / 'bin/javac').exists() else 'javac'
    java = str(java_home / 'bin/java') if (java_home / 'bin/java').exists() else 'java'
    subprocess.run([javac, '-encoding', 'UTF-8', '-d', str(root),
                    *map(str, root.rglob('*.java'))], check=True)
    subprocess.run([java, '-cp', str(root), 'PinpointResetCheck'], check=True)
