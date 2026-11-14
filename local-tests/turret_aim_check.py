#!/usr/bin/env python3
"""Check production turret geometry against lightweight FTC hardware substitutes."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
JAVA_ROOT = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
STUBS = {
    'com/acmerobotics/dashboard/config/Config.java':
        'package com.acmerobotics.dashboard.config; public @interface Config {}',
    'com/arcrobotics/ftclib/command/SubsystemBase.java':
        'package com.arcrobotics.ftclib.command; public class SubsystemBase {}',
    'com/qualcomm/robotcore/hardware/Servo.java': '''
        package com.qualcomm.robotcore.hardware;
        public class Servo {
          public enum Direction { FORWARD, REVERSE }
          public double position;
          public void setDirection(Direction d) {}
          public void setPosition(double p) { position = p; }
        }''',
    'com/qualcomm/robotcore/hardware/HardwareMap.java': '''
        package com.qualcomm.robotcore.hardware;
        public class HardwareMap {
          public <T> T get(Class<T> type, String name) { return type.cast(new Servo()); }
        }''',
    'org/firstinspires/ftc/teamcode/subsystem/LocalizationSubsystem.java': '''
        package org.firstinspires.ftc.teamcode.subsystem;
        public class LocalizationSubsystem {
          public static final double START_HEADING_DEG = 0;
          public double x, y, yaw; public boolean valid = true;
          public double getYawRadians() { return Math.toRadians(yaw); }
          public double getXIn() { return x; } public double getYIn() { return y; }
          public boolean isPinpointReady() { return true; }
          public boolean isFieldPoseValid() { return valid; }
        }''',
    'TurretAimCheck.java': '''
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import org.firstinspires.ftc.teamcode.subsystem.*;
        import org.firstinspires.ftc.teamcode.shoot.ShotTarget;
        import org.firstinspires.ftc.teamcode.shoot.TurretAimFeedback;
        public class TurretAimCheck {
          static void check(boolean ok, String label) {
            if (!ok) throw new AssertionError(label);
          }
          static void near(double actual, double expected, String label) {
            check(Math.abs(actual - expected) < 1e-8, label + ": " + actual);
          }
          public static void main(String[] args) {
            LocalizationSubsystem loc = new LocalizationSubsystem();
            TurretSubsystem turret = new TurretSubsystem(new HardwareMap(), false);
            check(turret.setAimMode(true, true), "enable aim");
            loc.y = -6;
            loc.yaw = -135; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), 0, "center");
            loc.yaw = -175; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), 40, "tracking");
            check(turret.hasValidAim() && turret.isReachable(), "reachable target");
            check(turret.setAimMode(false, false), "disable aim for center lock");
            turret.update(loc);
            near(turret.getPosition(), TurretSubsystem.INIT_POSITION, "center lock position");
            loc.yaw = -170; turret.update(loc);
            near(turret.getPosition(), TurretSubsystem.INIT_POSITION, "center lock follows chassis");
            check(turret.setAimMode(true, true), "resume aim");
            turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), 35, "tracking resumes");
            loc.yaw = -210; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), 60, "positive limit");
            check(!turret.isReachable(), "positive unreachable");
            loc.yaw = -60; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), -60, "negative limit");
            check(!turret.isReachable(), "negative unreachable");
            double previous = turret.getPosition();
            loc.valid = false; loc.yaw = 0; turret.update(loc);
            near(turret.getPosition(), previous, "hold on invalid pose");
            check(!turret.hasValidAim() && !turret.isReachable(), "invalid pose cannot aim");
            loc.valid = true; loc.yaw = -135; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), 0, "resume after valid pose");

            loc.x = 50; loc.y = 38; loc.yaw = 180; turret.update(loc);
            near(turret.getAimTargetX(), 60, "Hive X");
            near(turret.getAimTargetY(), 54, "low outer target extends coverage");
            check(turret.isReachable() && Math.toDegrees(Math.atan2(22, 10)) > 60,
                    "outer target reachable beyond center limit");
            loc.yaw = -55; turret.update(loc);
            near(turret.getAimTargetY(), 60, "fallback to the same Hive center");
            check(turret.isReachable(), "center remains reachable");

            loc.y = 106; loc.yaw = 180; turret.update(loc);
            near(turret.getAimTargetY(), 90, "high outer target extends coverage");
            check(turret.isReachable(), "mirrored outer target reachable");
            loc.x = 0; loc.y = 72; turret.update(loc);
            near(turret.getAimTargetY(), 90, "retain high Hive across center hysteresis");
            loc.y = 70; turret.update(loc);
            near(turret.getAimTargetY(), 54, "switch back to low Hive beyond hysteresis");
            loc.x = 60; loc.y = 54; loc.yaw = -90; turret.update(loc);
            near(turret.getAimTargetY(), 60, "fallback when directly at outer target");
            check(turret.hasValidAim() && turret.isReachable(), "nearby outer target uses center");

            ShotTarget chassis = new ShotTarget();
            chassis.update(50, 38, Math.PI);
            near(chassis.getTargetXInches(), 60, "chassis target X");
            near(chassis.getTargetYInches(), 54, "chassis low outer target");
            chassis.update(50, 106, Math.PI);
            near(chassis.getTargetYInches(), 90, "chassis high outer target");

            TurretAimFeedback feedback = new TurretAimFeedback();
            check(!feedback.update(true, 10, 0), "not aimed immediately");
            check(!feedback.update(true, 11, .19), "wait for settling");
            check(feedback.update(true, 11, .21), "one pulse after settling");
            check(feedback.isAimed(), "estimated aim true");
            check(!feedback.update(true, 12, .3), "no repeated pulse");
            check(!feedback.update(false, 12, .4) && !feedback.isAimed(), "invalid clears aim");
            check(!feedback.update(true, 12, .5), "reacquisition waits");
            check(feedback.update(true, 12, .71), "pulse on reacquisition");
            check(!feedback.update(true, 16, .8) && !feedback.isAimed(), "movement restarts settling");
            check(feedback.update(true, 16, 1.01), "pulse after movement settles");
            System.out.println("PASS: Hive aim points, turret coverage, center fallback, limits and feedback");
          }
        }''',
}

with tempfile.TemporaryDirectory(prefix='turret-aim-') as temp:
    root = Path(temp)
    for name, source in STUBS.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(source, encoding='utf-8')
    for name in ('NameConstants.java', 'subsystem/TurretSubsystem.java', 'subsystem/BlockerSubsystem.java',
                 'shoot/ShootConstants.java', 'shoot/ShotTarget.java', 'shoot/TurretAimFeedback.java'):
        target = root / 'org/firstinspires/ftc/teamcode' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(JAVA_ROOT / name, target)
    java_home = Path(os.environ.get('JAVA_HOME', ''))
    javac = str(java_home / 'bin/javac') if (java_home / 'bin/javac').exists() else 'javac'
    java = str(java_home / 'bin/java') if (java_home / 'bin/java').exists() else 'java'
    subprocess.run([javac, '-encoding', 'UTF-8', '-d', str(root),
                    *map(str, root.rglob('*.java'))], check=True)
    subprocess.run([java, '-cp', str(root), 'TurretAimCheck'], check=True)
