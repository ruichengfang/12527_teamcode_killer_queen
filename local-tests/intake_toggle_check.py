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
      public class HardwareMap { public Servo servo; public String requestedName;
        public <T> T tryGet(Class<T> type,String name) { requestedName=name; return type.cast(servo); } }''',
    'org/firstinspires/ftc/teamcode/NameConstants.java': '''
      package org.firstinspires.ftc.teamcode;
      public class NameConstants { public static final String flowerServo="flowerServo", PITCH_SERVO="pitchServo"; }''',
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
      public class ShooterSubsystem { public static double SHOT_TRANSIT_POWER=.8; public double target;
        public double getShotRpm(){return 3385.714285714286;} public void setTargetRpm(double rpm){target=rpm;}
        public void stop(){target=0;} public void update(double dt){} }''',
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
          cmd.initialize(); cmd.execute(); output(i,.7,0,"idle intake");
          g.left_trigger=1; cmd.execute(); output(i,1,1,"LT feeds both");
          g.left_trigger=0; cmd.execute(); output(i,.7,0,"release LT stops transit");
          g.left_bumper=true; cmd.execute(); output(i,-1,0,"LB reverses intake only");
          g.left_bumper=false; cmd.execute(); output(i,.7,0,"resume idle after reverse");
          g.right_trigger=1; cmd.execute(); output(i,0,0,"shoot waits for blocker");
          gate[0]=true; cmd.execute(); output(i,1,.8,"shoot after blocker");
          g.left_bumper=true; cmd.execute(); output(i,1,.8,"shoot overrides reverse");
          g.left_bumper=false; g.right_trigger=0; cmd.execute(); output(i,.7,0,"resume idle after shooting");
          g.dpad_down=true; cmd.execute(); output(i,-1,0,"dpad down reverses intake only");
          g.dpad_down=false; g.left_trigger=1; cmd.execute(); output(i,1,1,"LT feeds both again");
          cmd.end(true); output(i,0,0,"interruption stops");
          check(!cmd.isIntakeEnabled(),"interruption clears intake request");
          System.out.println("PASS: intake, transit, shot gating and stop");
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
          check("flowerServo".equals(h.requestedName),"flower servo configuration name");
          check(ShovelSubsystem.shovelDownPosition==.15 && ShovelSubsystem.shovelUpPosition==.55,"configured positions");
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
    'PitchAndShooterCheck.java': '''
      import com.qualcomm.robotcore.hardware.*;
      import org.firstinspires.ftc.teamcode.subsystem.*;
      import org.firstinspires.ftc.teamcode.command.ShooterCommand;
      public class PitchAndShooterCheck {
        static void near(double a,double b,String m){if(Math.abs(a-b)>1e-8)throw new AssertionError(m+": "+a);}
        public static void main(String[] args) {
          HardwareMap h=new HardwareMap(); h.servo=new Servo();
          PitchSubsystem pitch=new PitchSubsystem(h); near(h.servo.position,.52,"initial pitch");
          pitch.adjust(-1); near(h.servo.position,.51,"LB pitch");
          pitch.adjust(1); near(h.servo.position,.52,"RB pitch");
          PitchSubsystem absent=new PitchSubsystem(new HardwareMap()); absent.adjust(1);
          if(absent.isAvailable()) throw new AssertionError("missing pitch");
          ShooterSubsystem shooter=new ShooterSubsystem(); boolean[] wall={false}, rt={false};
          ShooterCommand cmd=new ShooterCommand(shooter,()->wall[0],()->rt[0],()->.02);
          cmd.execute(); near(shooter.target,3385.714285714286,"auto aim speed");
          wall[0]=true; cmd.execute(); near(shooter.target,2000,"wall standby");
          rt[0]=true; cmd.execute(); near(shooter.target,3450,"wall shot");
          rt[0]=false; cmd.execute(); near(shooter.target,2000,"release returns standby");
          wall[0]=false; cmd.execute(); near(shooter.target,3385.714285714286,"restore aim speed");
          cmd.end(true); near(shooter.target,0,"stop flywheel");
          System.out.println("PASS: pitch adjustments and shooter mode targets");
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
    for name in ('subsystem/PitchSubsystem.java', 'command/ShooterCommand.java'):
        target = root / 'org/firstinspires/ftc/teamcode' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(JAVA / name, target)
    java_home = Path(os.environ.get('JAVA_HOME', ''))
    javac = str(java_home / 'bin/javac') if (java_home / 'bin/javac').exists() else 'javac'
    java = str(java_home / 'bin/java') if (java_home / 'bin/java').exists() else 'java'
    subprocess.run([javac, '-encoding', 'UTF-8', '-d', str(root),
                    *map(str, root.rglob('*.java'))], check=True)
    subprocess.run([java, '-cp', str(root), 'IntakeToggleCheck'], check=True)
    subprocess.run([java, '-cp', str(root), 'ShovelCheck'], check=True)
    subprocess.run([java, '-cp', str(root), 'PitchAndShooterCheck'], check=True)
