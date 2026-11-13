#!/usr/bin/env python3
"""Check the production intake command with simple gamepad and motor substitutes."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
JAVA = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
STUBS = {
    'com/acmerobotics/dashboard/config/Config.java':
        'package com.acmerobotics.dashboard.config; public @interface Config {}',
    'com/arcrobotics/ftclib/command/SubsystemBase.java':
        'package com.arcrobotics.ftclib.command; public class SubsystemBase { public void periodic() {} }',
    'com/qualcomm/robotcore/hardware/Servo.java': '''
      package com.qualcomm.robotcore.hardware;
      public class Servo { public double position; public int writes;
        public void setPosition(double p) { position=p; writes++; } }''',
    'com/qualcomm/robotcore/hardware/HardwareMap.java': '''
      package com.qualcomm.robotcore.hardware;
      public class HardwareMap { public Servo servo;
        public <T> T tryGet(Class<T> type,String name) { return type.cast(servo); } }''',
    'org/firstinspires/ftc/teamcode/NameConstants.java': '''
      package org.firstinspires.ftc.teamcode;
      public class NameConstants { public static final String shovelServo="shovelServo"; }''',
    'com/arcrobotics/ftclib/command/CommandBase.java': '''
      package com.arcrobotics.ftclib.command;
      public class CommandBase {
        public void addRequirements(Object... s) {} public void initialize() {}
        public void execute() {} public void end(boolean interrupted) {}
      }''',
    'com/qualcomm/robotcore/hardware/Gamepad.java': '''
      package com.qualcomm.robotcore.hardware;
      public class Gamepad {
        public float left_trigger, right_trigger; public boolean left_bumper, dpad_down;
      }''',
    'org/firstinspires/ftc/teamcode/shoot/ShootConstants.java': '''
      package org.firstinspires.ftc.teamcode.shoot;
      public class ShootConstants {
        public static final double TRIGGER_THRESHOLD=.05, INTAKE_JAM_POWER=-1,
          TRANSIT_REVERSE_POWER=-1, INTAKE_ACTIVE_POWER=1, TRANSIT_FEED_POWER=1;
      }''',
    'org/firstinspires/ftc/teamcode/subsystem/ShooterSubsystem.java': '''
      package org.firstinspires.ftc.teamcode.subsystem;
      public class ShooterSubsystem { public static double SHOT_TRANSIT_POWER=.8; }''',
    'org/firstinspires/ftc/teamcode/subsystem/IntakeSubsystem.java': '''
      package org.firstinspires.ftc.teamcode.subsystem;
      public class IntakeSubsystem {
        public double intake, transit;
        public void setPowers(double a,double b) { intake=a;transit=b; }
        public void stop() { setPowers(0,0); }
      }''',
    'IntakeToggleCheck.java': '''
      import com.qualcomm.robotcore.hardware.Gamepad;
      import org.firstinspires.ftc.teamcode.command.IntakeCommand;
      import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
      public class IntakeToggleCheck {
        static void check(boolean b,String m){if(!b)throw new AssertionError(m);}
        static void output(IntakeSubsystem i,double a,double b,String m){
          check(Math.abs(i.intake-a)<1e-9 && Math.abs(i.transit-b)<1e-9,m);
        }
        public static void main(String[] args) {
          Gamepad g=new Gamepad(); IntakeSubsystem i=new IntakeSubsystem(); boolean[] gate={false};
          IntakeCommand cmd=new IntakeCommand(i,g,()->gate[0]);
          g.left_trigger=1; cmd.initialize(); cmd.execute(); output(i,1,0,"held at start runs intake only");
          g.left_trigger=0; cmd.execute(); g.left_trigger=1; cmd.execute(); output(i,1,1,"toggle on");
          cmd.execute(); output(i,1,1,"hold does not repeat");
          g.left_trigger=0; cmd.execute(); output(i,1,1,"release keeps intake and transit on");
          g.left_bumper=true; cmd.execute(); output(i,-1,-1,"reverse overrides");
          g.left_bumper=false; cmd.execute(); output(i,1,1,"resume after reverse");
          g.right_trigger=1; cmd.execute(); output(i,1,0,"shoot waits for blocker");
          gate[0]=true; cmd.execute(); output(i,1,.8,"shoot after blocker");
          g.right_trigger=0; cmd.execute(); output(i,1,1,"resume intake and transit after shooting");
          g.left_trigger=1; cmd.execute(); output(i,1,0,"toggle transit off with intake running");
          g.left_trigger=0; cmd.execute(); output(i,1,0,"full intake with transit off");
          g.left_trigger=1; cmd.execute(); output(i,1,1,"transit on again");
          cmd.end(true); output(i,0,0,"interruption stops");
          check(!cmd.isTransitEnabled(),"interruption clears transit latch");
          System.out.println("PASS: full intake, transit toggle, overrides and stop");
        }
      }''',
    'ShovelCheck.java': '''
      import com.qualcomm.robotcore.hardware.*;
      import org.firstinspires.ftc.teamcode.subsystem.ShovelSubsystem;
      public class ShovelCheck {
        static void check(boolean b,String m){if(!b)throw new AssertionError(m);}
        public static void main(String[] args) {
          HardwareMap h=new HardwareMap(); h.servo=new Servo();
          ShovelSubsystem shovel=new ShovelSubsystem(h);
          shovel.periodic(); check(h.servo.writes==0 && !shovel.hasTarget(),"no initial movement");
          ShovelSubsystem.shovelUpPosition=.7; ShovelSubsystem.shovelDownPosition=.3;
          shovel.toggle(); check(shovel.isShovelUp() && h.servo.position==.7,"first press up");
          shovel.toggle(); check(!shovel.isShovelUp() && h.servo.position==.3,"second press down");
          ShovelSubsystem.shovelDownPosition=.4; shovel.periodic();
          check(h.servo.position==.4,"dashboard position update");
          ShovelSubsystem missing=new ShovelSubsystem(new HardwareMap());
          missing.toggle(); missing.periodic(); check(!missing.isAvailable(),"missing servo tolerated");
          System.out.println("PASS: shovel toggle, configuration and missing device");
        }
      }''',
}

with tempfile.TemporaryDirectory(prefix='intake-toggle-') as temp:
    root = Path(temp)
    for name, content in STUBS.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')
    target = root / 'org/firstinspires/ftc/teamcode/command/IntakeCommand.java'
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(JAVA / 'command/IntakeCommand.java', target)
    target = root / 'org/firstinspires/ftc/teamcode/subsystem/ShovelSubsystem.java'
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(JAVA / 'subsystem/ShovelSubsystem.java', target)
    java_home = Path(os.environ.get('JAVA_HOME', ''))
    javac = str(java_home / 'bin/javac') if (java_home / 'bin/javac').exists() else 'javac'
    java = str(java_home / 'bin/java') if (java_home / 'bin/java').exists() else 'java'
    subprocess.run([javac, '-encoding', 'UTF-8', '-d', str(root),
                    *map(str, root.rglob('*.java'))], check=True)
    subprocess.run([java, '-cp', str(root), 'IntakeToggleCheck'], check=True)
    subprocess.run([java, '-cp', str(root), 'ShovelCheck'], check=True)
