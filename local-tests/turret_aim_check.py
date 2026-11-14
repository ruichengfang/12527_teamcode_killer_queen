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
          public static final double OPTION_RESET_X_IN = 72, OPTION_RESET_Y_IN = 72;
          public double x, y, yaw; public boolean valid = true;
          public double getYawRadians() { return Math.toRadians(yaw); }
          public double getXIn() { return x; } public double getYIn() { return y; }
          public boolean isPinpointReady() { return true; }
          public boolean isFieldPoseValid() { return valid; }
        }''',
    'TurretAimCheck.java': '''
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import org.firstinspires.ftc.teamcode.subsystem.*;
        import org.firstinspires.ftc.teamcode.shoot.SegmentedAutoAim;
        import org.firstinspires.ftc.teamcode.shoot.SegmentedAutoAimConstants;
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
            loc.y = -4;
            loc.yaw = -135; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), 0, "center");
            loc.yaw = -175; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), 40, "tracking");
            check(turret.hasValidAim() && turret.isReachable(), "reachable target");
            check(turret.setAimMode(false, false), "disable aim for center lock");
            turret.update(loc);
            near(turret.getPosition(), .48, "center lock position");
            loc.yaw = -170; turret.update(loc);
            near(turret.getPosition(), TurretSubsystem.INIT_POSITION, "center lock follows chassis");
            TurretSubsystem fixedTurret = new TurretSubsystem(new HardwareMap(), true);
            fixedTurret.update(loc);
            near(fixedTurret.getPosition(), .48, "fixed turret uses calibrated center");
            check(turret.setAimMode(true, true), "resume aim");
            turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), 35, "tracking resumes");
            loc.yaw = -210; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), 75, "tracking beyond old limit");
            check(turret.isReachable(), "75 degrees reachable");
            loc.yaw = -225; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), 90, "positive boundary");
            check(turret.isReachable(), "positive boundary reachable");
            loc.yaw = -235; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), 90, "positive limit");
            check(!turret.isReachable(), "positive unreachable");
            loc.yaw = -45; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), -90, "negative boundary");
            check(turret.isReachable(), "negative boundary reachable");
            loc.yaw = -35; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), -90, "negative limit");
            check(!turret.isReachable(), "negative unreachable");
            double previous = turret.getPosition();
            loc.valid = false; loc.yaw = 0; turret.update(loc);
            near(turret.getPosition(), previous, "hold on invalid pose");
            check(!turret.hasValidAim() && !turret.isReachable(), "invalid pose cannot aim");
            loc.valid = true; loc.yaw = -135; turret.update(loc);
            near(turret.getCommandedRelativeDegrees(), 0, "resume after valid pose");

            loc.x = 50; loc.y = 40; loc.yaw = 180; turret.update(loc);
            near(turret.getAimTargetX(), 60, "Hive X");
            near(turret.getAimTargetY(), 56, "low outer target extends coverage");
            check(turret.isReachable() && Math.toDegrees(Math.atan2(20, 10)) > 60,
                    "outer target reachable beyond center limit");
            loc.yaw = -25; turret.update(loc);
            near(turret.getAimTargetY(), 56, "low Hive target stays fixed when out of range");
            near(turret.getAimDistance(), Math.hypot(10, 16), "rotation does not change target distance");
            near(turret.getCommandedRelativeDegrees(), -90, "low Hive turn stops at the limit");
            check(!turret.isReachable(), "low Hive remains out of range");

            loc.y = 104; loc.yaw = 180; turret.update(loc);
            near(turret.getAimTargetY(), 88, "high outer target extends coverage");
            check(turret.isReachable(), "mirrored outer target reachable");
            loc.yaw = 25; turret.update(loc);
            near(turret.getAimTargetY(), 88, "high Hive target stays fixed when out of range");
            near(turret.getCommandedRelativeDegrees(), 90, "high Hive turn stops at the limit");
            check(!turret.isReachable(), "high Hive remains out of range");
            loc.x = 0; loc.y = 72; turret.update(loc);
            near(turret.getAimTargetY(), 88, "retain high Hive across center hysteresis");
            loc.y = 70; turret.update(loc);
            near(turret.getAimTargetY(), 56, "switch back to low Hive beyond hysteresis");
            loc.x = 60; loc.y = 56; loc.yaw = -90; turret.update(loc);
            near(turret.getAimTargetY(), 56, "coincident target stays on the selected Hive point");
            check(!turret.hasValidAim() && !turret.isReachable(), "coincident target has no aiming direction");

            ShotTarget chassis = new ShotTarget();
            chassis.update(50, 38, Math.PI);
            near(chassis.getTargetXInches(), 60, "chassis target X");
            near(chassis.getTargetYInches(), 56, "chassis low outer target");
            chassis.update(50, 106, Math.PI);
            near(chassis.getTargetYInches(), 88, "chassis high outer target");

            loc.x = 72; loc.y = 72; turret.update(loc);
            check(SegmentedAutoAim.isInCenter(loc), "center is blocked independently of table");
            loc.x = 54; loc.y = 90; turret.update(loc);
            check(SegmentedAutoAim.isInCenter(loc), "center includes boundary");
            loc.x = 53; loc.y = 90; turret.update(loc);
            check(!SegmentedAutoAim.isInCenter(loc), "outside center boundary");
            double[][] intervals = {{0, 26, 3000, .50}, {26, 37.5, 3200, .51}, {37.5, -1, 3400, .52}};
            SegmentedAutoAimConstants.shotTable = intervals;
            check(SegmentedAutoAim.select(loc, turret) == intervals[0], "near interval");
            loc.x = 28; loc.y = 90; loc.yaw = 180; turret.update(loc);
            check(SegmentedAutoAim.select(loc, turret) == intervals[1], "middle interval");
            loc.x = 17; turret.update(loc);
            check(SegmentedAutoAim.select(loc, turret) == intervals[2], "wall distance interval");
            loc.yaw = 130; turret.update(loc);
            double[] wideAngleProfile = SegmentedAutoAim.select(loc, turret);
            check(wideAngleProfile == intervals[2], "angle does not change row");
            near(wideAngleProfile[SegmentedAutoAim.RPM], 3400, "far RPM from fixture");
            near(wideAngleProfile[SegmentedAutoAim.PITCH], .52, "far pitch from fixture");
            loc.valid = false; turret.update(loc);
            check(SegmentedAutoAim.select(loc, turret) == null, "invalid pose has no profile");

            check(SegmentedAutoAim.findProfileForDistance(0) == intervals[0], "zero belongs to first interval");
            check(SegmentedAutoAim.findProfileForDistance(26) == intervals[0], "26 belongs to near interval");
            check(SegmentedAutoAim.findProfileForDistance(26.01) == intervals[1], "just above 26");
            check(SegmentedAutoAim.findProfileForDistance(37.5) == intervals[1], "37.5 belongs to middle interval");
            check(SegmentedAutoAim.findProfileForDistance(37.51) == intervals[2], "just above 37.5");
            check(SegmentedAutoAim.findProfileForDistance(100) == intervals[2], "unbounded final interval");

            SegmentedAutoAimConstants.shotTable = new double[][] {
                {60, 90, 3900, .60}, {10, 30, 2600, .45}, {40, 60, 3200, .50}, {55, 9999}, null
            };
            double[] profile = SegmentedAutoAim.findProfileForDistance(55);
            near(profile[SegmentedAutoAim.RPM], 3200, "interval supplies RPM");
            near(profile[SegmentedAutoAim.PITCH], .50, "same row supplies pitch");
            check(SegmentedAutoAim.findProfileForDistance(35) == null, "gap has no profile");
            check(SegmentedAutoAim.findProfileForDistance(2) == null, "below table range");
            check(SegmentedAutoAim.findProfileForDistance(100) == null, "above bounded table");
            check(SegmentedAutoAim.findProfileForDistance(Double.NaN) == null, "invalid distance");
            SegmentedAutoAimConstants.shotTable = new double[0][];
            check(SegmentedAutoAim.findProfileForDistance(50) == null, "empty table has no profile");

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
            System.out.println("PASS: Hive aim points, turret limits, distance table, center block and feedback");
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
                 'shoot/ShootConstants.java', 'shoot/ShotTarget.java', 'shoot/SegmentedAutoAim.java',
                 'shoot/SegmentedAutoAimConstants.java', 'shoot/TurretAimFeedback.java'):
        target = root / 'org/firstinspires/ftc/teamcode' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(JAVA_ROOT / name, target)
    java_home = Path(os.environ.get('JAVA_HOME', ''))
    javac = str(java_home / 'bin/javac') if (java_home / 'bin/javac').exists() else 'javac'
    java = str(java_home / 'bin/java') if (java_home / 'bin/java').exists() else 'java'
    subprocess.run([javac, '-encoding', 'UTF-8', '-d', str(root),
                    *map(str, root.rglob('*.java'))], check=True)
    subprocess.run([java, '-cp', str(root), 'TurretAimCheck'], check=True)
