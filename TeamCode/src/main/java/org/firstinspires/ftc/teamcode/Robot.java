package org.firstinspires.ftc.teamcode;

// IMPORT SUBSYSTEMS

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.Component;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;
import org.firstinspires.ftc.teamcode.lib.MotorEx;
import org.firstinspires.ftc.teamcode.lib.RobotFlags;
import org.firstinspires.ftc.teamcode.lib.StepperServo;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmElbow;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Claw;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSensor;
import org.firstinspires.ftc.teamcode.subsystems.relocalization.Relocalization;
import org.firstinspires.ftc.teamcode.subsystems.slides.Slides;
import org.firstinspires.ftc.teamcode.subsystems.launcher.DroneLauncher;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;

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
    public Levels subsystemState = Levels.INTAKE;
    public boolean intaking = false;
    public ArrayList<RobotFlags> flags = new ArrayList<>();


    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        this.drive = new SampleMecanumDrive(map);

//        this.cv = new CVMaster(map);
        //this.intake.intakeMotor = map.get(DcMotorEx.class, "intakeMotor");
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
                new StepperServo(1, "claw", map),                 //11
                new StepperServo(1, "wrist", map),                   //12
                new MotorEx(1, "intakeMotor", map, false),   //13
                new StepperServo(1, "intakeServo1", map),            //14
                new StepperServo(1, "intakeServo2", map),            //15

                new StepperServo(1, "drone", map),                   //16

        };

        VoltageSensor voltageSensor = map.voltageSensor.iterator().next();

        // INIT SUBSYSTEMS

        this.claw = new Claw((StepperServo) components[11], (StepperServo) components[12]);
        this.arm = new ArmElbow((StepperServo) components[8], (StepperServo) components[9], (StepperServo) components[10]);
        this.intake = new Intake((StepperServo) components[14], (StepperServo) components[15], (MotorEx) components[13]);
//        this.intakeSensor = new IntakeSensor(map.get(NormalizedColorSensor.class, "intakeSensor1"), map.get(NormalizedColorSensor.class, "intakeSensor2"), 2);
        this.slides = new Slides((Motor) components[4], (Motor) components[5], (Motor) components[6], (StepperServo) components[7], voltageSensor);
        this.drone = new DroneLauncher((StepperServo) components[16]);
        this.relocalization = new Relocalization(map);
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
        this.intake.startIntake();
        this.arm.setAngleArm(27);
        this.claw.setPositionClaw(185);
        this.intake.setAngle(197);
        this.claw.wrist.setAngle(123);
        this.arm.setAngleElbow(110);
        this.slides.runToPosition(0);
    }

    public void stopIntake() {
        intaking = false;
        this.arm.setAngleArm(0);
        this.arm.setAngleElbow(119);
        Thread thread = new Thread(new Runnable() {
            public void run() {
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                claw.setPositionClaw(245);
                intake.stopIntake();
                intake.setAngle(130);
                try {
                    Thread.sleep(600);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                arm.setAngleArm(30);
                arm.setAngleElbow(115);
            }});
        thread.start();
        subsystemState = Levels.INTERMEDIATE;
    }

    public void autoIntake() {
        intaking = false;
        this.arm.setAngleArm(0);
        this.arm.setAngleElbow(112);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        this.claw.setPositionClaw(250);
        this.intake.stopIntake();
        this.intake.setAngle(120);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        this.arm.setAngleArm(30);
        this.arm.setAngleElbow(115);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        this.intake.setAngle(50);
        this.subsystemState = Levels.INTERMEDIATE;
    }

    /**
    * <h1>WARNING: BLOCKS THREAD</h1>
     * Blocks thread until intake is complete with specified number of pixels (1-2)
    */
    public void startSmartIntake(int pixels) {
        this.startIntake();
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

    public void smartClawOpen() {
        this.claw.setClawOpen();
        Thread thread = new Thread(new Runnable() {
            public void run() {
                try {
                    Thread.sleep(300);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                arm.setAngleArm(30);
                arm.setAngleElbow(110);
                claw.wrist.setAngle(123);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                slides.runToPosition(0);
            }});
        thread.start();
    }

    public void smartIntakeUpdate() {
        if (intaking) {
            boolean[] state = intakeSensor.hasPixel();
            if (state[0] && state[1]){
                this.stopIntake();
                intaking = false;
            }
        }
    }

    public void autoIntake(long time, float intakeAngle) {
        intaking = true;
        this.intake.startIntake();
        this.arm.setAngleArm(26);
        this.claw.setPositionClaw(200);
        this.intake.setAngle(intakeAngle);
        this.claw.wrist.setAngle(123);
        this.arm.setAngleElbow(110);
        this.slides.runToPosition(0);

        try {
            Thread.sleep(time*1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        this.stopIntake();
    }

    public void depositPreset() {
        this.slides.runToPreset(Levels.DEPOSIT);
        Thread thread = new Thread(new Runnable() {
            public void run() {
                try {
                    Thread.sleep(700);
                } catch (Exception e) {
                }
                arm.runtoPreset(Levels.DEPOSIT);
                claw.wrist.setAngle(123);
            }});
        thread.start();
        this.subsystemState = Levels.DEPOSIT;
    }

    public void autoPreloadDepositPreset() {
        this.slides.runToPosition(200);
        Thread thread = new Thread(new Runnable() {
            public void run() {
                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                }
                arm.runtoPreset(Levels.DEPOSIT);
                claw.wrist.setAngle(123);
            }});
        thread.start();
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
        this.flags.add(RobotFlags.CLIMB_ENGAGED);
        this.flags.add(RobotFlags.CLIMB_EXTEND_IN_PROGRESS);
        this.slides.runToClimb();
        this.intake.setAngle(130);
        Thread thread = new Thread(new Runnable() {
            public void run() {
                try {
                    Thread.sleep(250);
                } catch (Exception e) {
                }
                arm.runtoPreset(Levels.DEPOSIT);
                claw.wrist.setAngle(123);
                while (slides.getPos() <= 460 && !flags.contains(RobotFlags.CLIMB_RETRACT_REQUESTED)) {
                    // sleep
                }
                if (!flags.contains(RobotFlags.CLIMB_RETRACT_REQUESTED)) {
                    slides.setPower((float) 0.6);
                    slides.shiftGear(true);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    slides.setPower(0);
                }

                if (flags.contains(RobotFlags.CLIMB_RETRACT_REQUESTED)) {
                    slides.shiftGear(false);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    smartClawOpen();
                    flags.remove(RobotFlags.CLIMB_RETRACT_REQUESTED);
                }
                flags.remove(RobotFlags.CLIMB_EXTEND_IN_PROGRESS);
            }});
        thread.start();
    }

    /**
     * <h1>CLIMB DISENGAGE, NOT CLIMB START</h1>
     */
    public void climbRetract() {
        this.flags.remove(RobotFlags.CLIMB_ENGAGED);
        if (flags.contains(RobotFlags.CLIMB_EXTEND_IN_PROGRESS)) {
            flags.add(RobotFlags.CLIMB_RETRACT_REQUESTED);
        } else {
            slides.shiftGear(false);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            smartClawOpen();
        }
    }

    public void antiJam(){
        if (intaking) {
            if (this.intake.intakeMotor.getCurrent() > 5.0) {
                this.intake.setAngle(100);
                this.intake.reverse();
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                this.intake.setAngle(197);
                this.intake.intakeMotor.setSpeed(0.6F);
            }
        }
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
        this.subsystemState = Levels.CLIMB;
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
