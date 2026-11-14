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
    'org/firstinspires/ftc/teamcode/subsystem/ShooterSubsystem.java': '''
        package org.firstinspires.ftc.teamcode.subsystem;
        public class ShooterSubsystem {
          public static final double TARGET_CHANGE_RESET_RPM = 30.0 * 60.0 / 28.0;
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
            checkProfile(SegmentedAutoAim.select(loc, turret), 3305, .49, "below measured range");
            loc.x = 15; loc.y = 88; loc.yaw = 180; turret.update(loc);
            double pitchAt45 = .50 + ((45 - 42.6) / (47.3 - 42.6)) * .03;
            double rpmAt45 = 3365 + ((45 - 42.6) / (47.3 - 42.6)) * 10;
            checkProfile(SegmentedAutoAim.select(loc, turret), rpmAt45, pitchAt45, "interpolated pose");
            loc.yaw = 130; turret.update(loc);
            checkProfile(SegmentedAutoAim.select(loc, turret), rpmAt45, pitchAt45, "angle alone does not change output");
            loc.valid = false; turret.update(loc);
            check(SegmentedAutoAim.select(loc, turret) == null, "invalid pose has no profile");

            for (double[] sample : new double[][] {
                {32, 3305, .49}, {37.2, 3315, .50}, {42.6, 3365, .50},
                {47.3, 3375, .53}, {51.3, 3475, .54}, {62.5, 3575, .53}
            }) {
                double[] exact = SegmentedAutoAim.findProfileForDistance(sample[0]);
                check(exact[SegmentedAutoAim.RPM] == sample[1] && exact[SegmentedAutoAim.PITCH] == sample[2],
                        "exact measured output at " + sample[0]);
            }
            checkProfile(SegmentedAutoAim.findProfileForDistance(34.6), 3310, .495, "near interpolation");
            checkProfile(SegmentedAutoAim.findProfileForDistance(39.9), 3340, .50, "flat pitch interpolation");
            checkProfile(SegmentedAutoAim.findProfileForDistance(56.9), 3525, .535, "decreasing pitch interpolation");
            checkProfile(SegmentedAutoAim.findProfileForDistance(100), 3575, .53, "above measured range");

            SegmentedAutoAimConstants.shotTable = new double[][] {
                {60, 3900, .60}, {10, 2600, .45}, {40, 3200, .50}, {55, 9999}, null
            };
            checkProfile(SegmentedAutoAim.findProfileForDistance(50), 3550, .55, "unordered table, invalid row skipped");
            double[] edited = SegmentedAutoAim.findNearestCalibrationPoint(49);
            check(edited == SegmentedAutoAimConstants.shotTable[2], "edit references the nearest measured row");
            edited[SegmentedAutoAim.RPM] += 10;
            checkProfile(SegmentedAutoAim.findProfileForDistance(40), 3210, .50, "point edit affects exact output");
            checkProfile(SegmentedAutoAim.findProfileForDistance(50), 3555, .55, "point edit affects interpolation");
            check(SegmentedAutoAim.findProfileForDistance(Double.NaN) == null, "invalid distance");
            SegmentedAutoAimConstants.shotTable = new double[0][];
            check(SegmentedAutoAim.findProfileForDistance(50) == null, "empty table has no profile");
            check(!SegmentedAutoAim.hasFeedTargetChanged(3306, .491, 3305, .49), "small pose jitter preserves feed");
            check(SegmentedAutoAim.hasFeedTargetChanged(3375, .49, 3305, .49), "large speed change rechecks feed");
            check(SegmentedAutoAim.hasFeedTargetChanged(3305, .52, 3305, .49), "large pitch change rechecks feed");

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
            System.out.println("PASS: Hive aim, limits, six calibration points, interpolation and feed thresholds");
          }

          static void checkProfile(double[] profile, double rpm, double pitch, String label) {
            near(profile[SegmentedAutoAim.RPM], rpm, label + " RPM");
            near(profile[SegmentedAutoAim.PITCH], pitch, label + " pitch");
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
