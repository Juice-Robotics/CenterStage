package org.firstinspires.ftc.teamcode;

// IMPORT SUBSYSTEMS

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.Component;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;
import org.firstinspires.ftc.teamcode.lib.StepperServo;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmElbow;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Claw;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSensor;
import org.firstinspires.ftc.teamcode.subsystems.relocalization.Relocalization;
import org.firstinspires.ftc.teamcode.subsystems.slides.Slides;
import org.firstinspires.ftc.teamcode.subsystems.launcher.DroneLauncher;

public class Robot {

    // SUBSYSTEM DECLARATIONS
    public Component[] components;
    public SampleMecanumDrive drive;
    public Claw claw;
    public ArmElbow arm;
    public Intake intake;
    public IntakeSensor intakeSensor;
    public Slides slides;
    public DroneLauncher drone;
    public Relocalization relocalization;
    public HardwareMap hardwareMap;
    public double CURRENT_HIGH = 1;
    public double ENCODER_MAX_DIFFERENCE = 1;

    // STATE VARS
    boolean auton;
    Levels subsystemState;
    public boolean intaking = false;


    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        this.drive = new SampleMecanumDrive(map);

//        this.cv = new CVMaster(map);

        this.components = new Component[]{
                new Motor(3, "leftRear", map, true),          //0 left odometer
                new Motor(2, "rightRear", map, false),        //1 right odometer
                new Motor(1, "leftFront", map, true),         //2 middle odometer
                new Motor(0, "rightFront", map, false),       //3

                new Motor(0, "slides1", map, false),          //4
                new Motor(0, "slides2", map, false),          //5
                new Motor(0, "climb", map, false),            //6
                new StepperServo(1, "shifter", map),                 //7

                new StepperServo(1, "arm1", map),                    //8
                new StepperServo(1, "arm2", map),                    //9
                new StepperServo(1, "elbow", map),                   //10
                new StepperServo(1, "deposit", map),                 //11
                new StepperServo(1, "wrist", map),                   //12

                new Motor(0, "intakeMotor", map, false),      //13
                new StepperServo(1, "intakeServo1", map),            //14
                new StepperServo(1, "intakeServo2", map),            //15

                new StepperServo(1, "drone", map),                   //16

        };

        VoltageSensor voltageSensor = map.voltageSensor.iterator().next();

        // INIT SUBSYSTEMS

        this.claw = new Claw((StepperServo) components[11], (StepperServo) components[12]);
        this.arm = new ArmElbow((StepperServo) components[8], (StepperServo) components[9], (StepperServo) components[10]);
        this.intake = new Intake((StepperServo) components[14], (StepperServo) components[15], (DcMotorEx) components[13]);
        this.intakeSensor = new IntakeSensor(map.get(NormalizedColorSensor.class, "intakeSensor1"), map.get(NormalizedColorSensor.class, "intakeSensor2"), 2);
        this.slides = new Slides((Motor) components[4], (Motor) components[5], (Motor) components[6], (StepperServo) components[7], voltageSensor);
        this.hardwareMap = map;

        this.subsystemState = Levels.ZERO;
    }

    // INTAKE
    public void intakePreset() {
        //turning to get through the thingy
        this.slides.runToPreset(Levels.INTAKE);
        this.arm.runtoPreset(Levels.INTAKE);
        this.claw.setClawOpen();
        this.claw.wrist.setAngle(123);
        this.intake.setAngle(192);
        this.subsystemState = Levels.INTAKE;
    }

    public void startIntake() {
        intaking = true;
        this.claw.setClawOpen();
        this.arm.setAngleArm(0);
        this.arm.setAngleElbow(112);
        this.claw.wrist.setAngle(123);
        this.intake.setAngle(192);
        this.arm.setAngleArm(6);
        this.intake.startIntake();
    }

    public void stopIntake() {
        intaking = false;
        this.arm.setAngleArm(0);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        this.claw.setPositionClaw(215);
        this.intake.stopIntake();
        this.intake.setAngle(130);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        this.arm.setAngleArm(15);
        this.arm.setAngleElbow(115);
    }

    /**
    * <h1>WARNING: BLOCKS THREAD</h1>
     * Blocks thread until intake is complete with specified number of pixels (1-2)
    */
    public void startSmartIntake(int pixels) {
        this.intake.startIntake();
        if (pixels == 1) {
            while (!intakeSensor.hasPixel()[0] && !intakeSensor.hasPixel()[1]) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        } else if (pixels == 2) {
            while (!intakeSensor.hasPixel()[0] || !intakeSensor.hasPixel()[1]) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
        this.intake.stopIntake();
    }

    public void smartIntake(boolean[] state) {
        if (state[0] && state[1]){
            intake.stopIntake();
            intaking = false;
        }
    }

    public void depositSmart(boolean[] state) {
        if (state[0] && state[1] && (this.subsystemState == Levels.INTAKE)){
            claw.setClawClose();
        }
    }

    public void smartIntakeUpdate() {
        if (intaking) {
            boolean[] state = intakeSensor.hasPixel();
            depositSmart(state);
            smartIntake(state);
        }
    }

    public void depositPreset() {
        this.claw.setPositionWrist(90); //turning to get through the thingy
        this.slides.runToPreset(Levels.DEPOSIT);
        this.arm.runtoPreset(Levels.DEPOSIT);
        this.subsystemState = Levels.DEPOSIT;
    }
    public void runToAutoSpikePreset() {
        this.slides.runToPosition(100);
        this.arm.runtoPreset(Levels.DEPOSIT);
        this.arm.runtoPreset(Levels.DEPOSIT);
    }

    public void runToAutoBackdropPreset() {
        this.slides.runToPosition(100);
        this.arm.runtoPreset(Levels.BACKDROP);
    }

    public void climbExtend() {
        this.slides.runToClimb();
        this.arm.runtoPreset(Levels.BACKDROP);
    }
    public double checkJam(double previousPosition){
        if ((this.intake.intakeMotor.getCurrent(CurrentUnit.AMPS)> CURRENT_HIGH) && (this.intake.intakeMotor.getCurrentPosition()-previousPosition < ENCODER_MAX_DIFFERENCE)) {
            this.intake.reverse();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            this.intake.startIntake();
        }
        return this.intake.intakeMotor.getCurrentPosition();
    }
//    public void depositToIntake(){
//        this.arm.setAngleElbow(125);
//        this.arm.setAngleArm(15);
//        this.intake.setAngle(120);
//        this.claw.setPositionClaw(200);
//        this.arm.setAngleArm(0);
//        this.arm.setAngleElbow(112);
//        this.claw.wrist.setAngle(123);
//        this.intake.setAngle(192);
//        this.claw.setPositionClaw(140);
//        this.arm.setAngleArm(6);
//        this.intake.startIntake();
//    }

    public void startClimb() {
        this.slides.startClimb();
    }
//    public void intakeToReady() {
//        this.arm.setAngleArm(0);
//        try {
//            Thread.sleep(250);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        this.claw.setPositionClaw(215);
//        this.intake.stopIntake();
//        this.intake.setAngle(130);
//        try {
//            Thread.sleep(250);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        this.arm.setAngleArm(15);
//        this.arm.setAngleElbow(115);
//    }


    //DRIVE
    public void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = (y - x + rx) * -1;
        double powerBackRight = (y + x - rx) * -1;

        if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 ||
                Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
            max = Math.max(Math.abs(powerFrontRight), max);
            max = Math.max(Math.abs(powerBackRight), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            powerFrontLeft /= max;
            powerBackLeft /= max;
            powerFrontRight /= max;
            powerBackRight /= max;
        }
        Motor backLeft = (Motor) components[0];
        Motor backRight = (Motor) components[1];
        Motor frontLeft = (Motor) components[2];
        Motor frontRight = (Motor) components[3];
        frontLeft.setSpeed((float)powerFrontLeft);
        frontRight.setSpeed((float)powerFrontRight);
        backLeft.setSpeed(-(float)powerBackLeft);
        backRight.setSpeed(-(float)powerBackRight);
    }
}
