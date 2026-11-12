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
    .replace('public void resetPosAndIMU(){resets++;} public void update(){}',
             'public boolean failAfterSet, armed; public void resetPosAndIMU(){resets++;} '
             'public void update(){if(armed){status=DeviceStatus.FAULT_BAD_READ;armed=false;}}')
    .replace('public void setPosition(Pose2D p){pose=p;}',
             'public void setPosition(Pose2D p){pose=p;if(failAfterSet){armed=true;failAfterSet=false;}}'))
stubs['PinpointResetCheck.java'] = '''
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.command.PinpointTrimButtons;
public class PinpointResetCheck {
  static void check(boolean ok, String label) { if(!ok) throw new AssertionError(label); }
  static void near(double a,double b,String label) { check(Math.abs(a-b)<1e-8,label+": "+a); }
  public static void main(String[] args) {
    HardwareMap hw=new HardwareMap();
    IMU imu=new IMU(); GoBildaPinpointDriver od=new GoBildaPinpointDriver();
    hw.devices.put(NameConstants.IMU,imu); hw.devices.put(NameConstants.PINPOINT,od);
    LocalizationSubsystem loc=new LocalizationSubsystem(hw);
    loc.update();
    od.pose=new Pose2D(DistanceUnit.INCH,10,20,AngleUnit.DEGREES,30);
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
    check(!loc.adjustFieldPosition(1,0),"reject trim without valid pose");
    od.status=GoBildaPinpointDriver.DeviceStatus.READY; loc.update();
    near(loc.getXIn(),72,"invalid trim did not change x");
    HardwareMap missing=new HardwareMap(); missing.devices.put(NameConstants.IMU,new IMU());
    check(!new LocalizationSubsystem(missing).resetToFieldCenter(),"missing Pinpoint rejected");
    System.out.println("PASS: Pinpoint reset, trim buttons, invalid pose and driver heading");
  }
}'''
with tempfile.TemporaryDirectory(prefix='pinpoint-reset-') as temp:
    root = Path(temp)
    for name, content in stubs.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')
    for name in ('NameConstants.java', 'AutoEndPose.java', 'subsystem/LocalizationSubsystem.java',
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
