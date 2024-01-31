package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class ArmElbow {
    public StepperServo arm1;
    public StepperServo arm2;
    public StepperServo elbow;

    private MotionProfile profile;
    public MotionState curState;
    private ElapsedTime timer;
    double maxvel = 4000;
    double maxaccel = 4000;

    public double currentAngle;
    public double armTarget;

    // TARGETS
    public double OFFSET = 4;
    public double intakeTargetArm = 26;

    public double captureTargetArm = 16;
    public double depositTargetArm = 148;
    public double intakeTargetElbow = 110-OFFSET;
    public double depositTargetElbow = 217-OFFSET;
    public double initPos = 140;


    public ArmElbow(StepperServo arm1, StepperServo arm2, StepperServo elbow) {
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.elbow = elbow;

//        timer = new ElapsedTime();
//        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), maxvel, maxaccel);
    }

    public void setAngleArm(double angle) {
//        this.armTarget = angle;
//        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(currentAngle, 0), new MotionState(armTarget, 0), maxvel, maxaccel);
//        timer.reset();
        arm1.setAngle((float) angle);
        arm2.setAngle((float) angle);
    }

    public void setAngleElbow(double angle) {
        this.elbow.setAngle((float) angle);
        this.currentAngle = angle;
    }

    public double getAngle() {
        return currentAngle;
    }

    public void runtoPreset(Levels level){
        if (level == Levels.INTAKE) {
            this.setAngleArm(intakeTargetArm);
            this.setAngleElbow(intakeTargetElbow);
        } else if (level == Levels.ZERO){
            this.setAngleArm(intakeTargetArm);
            this.setAngleElbow(intakeTargetElbow);
        }
        else if (level == Levels.DEPOSIT) {
            this.setAngleArm(depositTargetArm);
            this.setAngleElbow(depositTargetElbow);
        }
        else if (level == Levels.INTERMEDIATE) {
            this.setAngleArm(33);
            this.setAngleElbow(115-OFFSET);
        }
        else if (level == Levels.CLIMB_EXTEND) {
            this.setAngleArm(160);
            this.setAngleElbow(237-OFFSET);
        }
        else if (level == Levels.CAPTURE) {
            this.setAngleArm(captureTargetArm);
            this.setAngleElbow(116-OFFSET);
        }
        else if (level == Levels.INIT) {
            this.setAngleArm(initPos);
            this.setAngleElbow(110-OFFSET);
        }
    }

//    public void update() {
//        MotionState state = profile.get(timer.time());
//        double tTarget = state.getX();
//
//        arm1.setAngle((float) tTarget);
//        arm2.setAngle((float) tTarget);
//    }

}
