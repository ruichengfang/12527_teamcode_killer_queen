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
          g.left_trigger=1; cmd.initialize(); cmd.execute(); output(i,0,0,"held at start");
          g.left_trigger=0; cmd.execute(); g.left_trigger=1; cmd.execute(); output(i,1,1,"toggle on");
          cmd.execute(); output(i,1,1,"hold does not repeat");
          g.left_trigger=0; cmd.execute(); output(i,1,1,"release stays on");
          g.left_bumper=true; cmd.execute(); output(i,-1,-1,"reverse overrides");
          g.left_bumper=false; cmd.execute(); output(i,1,1,"resume after reverse");
          g.right_trigger=1; cmd.execute(); output(i,1,0,"shoot waits for blocker");
          gate[0]=true; cmd.execute(); output(i,1,.8,"shoot after blocker");
          g.right_trigger=0; cmd.execute(); output(i,1,1,"resume intake after shooting");
          g.left_trigger=1; cmd.execute(); output(i,0,0,"toggle off");
          g.left_trigger=0; cmd.execute(); g.left_trigger=1; cmd.execute(); output(i,1,1,"on again");
          cmd.end(true); output(i,0,0,"interruption stops");
          check(!cmd.isIntakeEnabled(),"interruption clears latch");
          System.out.println("PASS: intake toggle, overrides, shooting and stop");
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
    java_home = Path(os.environ.get('JAVA_HOME', ''))
    javac = str(java_home / 'bin/javac') if (java_home / 'bin/javac').exists() else 'javac'
    java = str(java_home / 'bin/java') if (java_home / 'bin/java').exists() else 'java'
    subprocess.run([javac, '-encoding', 'UTF-8', '-d', str(root),
                    *map(str, root.rglob('*.java'))], check=True)
    subprocess.run([java, '-cp', str(root), 'IntakeToggleCheck'], check=True)
