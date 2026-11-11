import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.autopath.*;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Exercise the actual autonomous entrypoints with deterministic lifecycle/hardware boundaries. */
public final class AutoLifecycleCheck {
    static void full(int ticks,boolean abort) {
        AutoEndPose.save(99,99,0);
        AutoPath_full op=new AutoPath_full();op.hardwareMap=BehaviorReplay.hardware(true);
        op.activeTicks=ticks;op.abortBeforeStart=abort;op.runOpMode();
        if(abort) {
            LifecycleCheck.check(!AutoEndPose.hasValue(),"INIT abort must clear stale pose without saving a new one");
        } else {
            LifecycleCheck.check(AutoEndPose.hasValue(),"full auto saves pose at STOP");
            LifecycleCheck.close(AutoEndPose.getXIn(),ticks==1?60:15,"full saved actual x");
            LifecycleCheck.close(AutoEndPose.getYIn(),ticks==1?20.7869:110,"full saved actual y");
            LifecycleCheck.close(AutoEndPose.getHeadingRad(),Math.toRadians(ticks==1?90:180),"saved heading radians");
        }
        LifecycleCheck.check(Constants.last.events.contains("stop")&&Constants.last.events.contains("motorStop"),
            "full auto cleans up follower and drivetrain");
    }

    static void formal(int ticks,boolean abort) {
        AutoEndPose.save(99,99,0);
        AutoFormal op=new AutoFormal();op.hardwareMap=BehaviorReplay.hardware(true);
        op.activeTicks=ticks;op.abortBeforeStart=abort;op.runOpMode();
        if(abort) {
            LifecycleCheck.check(!AutoEndPose.hasValue(),"Formal INIT abort must not save pose");
        } else {
            LifecycleCheck.check(AutoEndPose.hasValue(),"Formal saves actual pose");
            LifecycleCheck.close(AutoEndPose.getXIn(),ticks==1?57.1091:8.6715,"Formal saved x");
            LifecycleCheck.close(AutoEndPose.getYIn(),ticks==1?21.6998:107.8308,"Formal saved y");
            LifecycleCheck.close(AutoEndPose.getHeadingRad(),Math.toRadians(ticks==1?90:0),"Formal heading");
            if(ticks>=8) LifecycleCheck.check(Constants.last.events.stream()
                .filter(e->e.startsWith("follow:")).count()==8,"Formal executes all eight paths");
        }
        LifecycleCheck.check(Constants.last.events.contains("stop")&&Constants.last.events.contains("motorStop"),
            "Formal cleanup");
    }

    static void demo(boolean abort,boolean pinpoint,int ticks) {
        AutoEndPose.save(99,99,Math.PI);
        AutoPath_demo op=new AutoPath_demo();op.hardwareMap=BehaviorReplay.hardware(pinpoint);
        op.abortBeforeStart=abort;op.activeTicks=ticks;op.runOpMode();
        LifecycleCheck.allMotorsStopped(op.hardwareMap);
        LifecycleCheck.check(AutoEndPose.hasValue()==(!abort&&pinpoint),"demo saves only initialized pose after START");
        if(!abort&&pinpoint) {
            LifecycleCheck.close(AutoEndPose.getXIn(),8,"demo owns its start x");
            LifecycleCheck.close(AutoEndPose.getYIn(),111,"demo owns its start y");
            LifecycleCheck.close(AutoEndPose.getHeadingRad(),0,"demo ignores previous heading");
        }
        LifecycleCheck.check(op.hardwareMap.get(DcMotorEx.class,NameConstants.FRONT_LEFT).direction==DcMotor.Direction.REVERSE,
            "demo direction matches jrc");
    }

    public static void main(String[] args) {
        full(6,false);full(1,false);full(0,true);
        formal(10,false);formal(1,false);formal(0,true);
        demo(false,true,30);demo(false,true,1);demo(true,true,0);demo(false,false,30);
        AutoEndPose.clear();
        System.out.println("PASS: actual auto entrypoints, final/early-stop pose save, INIT abort, demo explicit start, missing Pinpoint, motor cleanup");
    }
}
