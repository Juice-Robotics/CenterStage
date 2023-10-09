package org.firstinspires.ftc.teamcode;

// IMPORT SUBSYSTEMS

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.Component;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;
import org.firstinspires.ftc.teamcode.lib.StepperServo;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWrist;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.relocalization.Relocalization;
import org.firstinspires.ftc.teamcode.subsystems.slides.Slides;
import org.firstinspires.ftc.teamcode.subsystems.launcher.DroneLauncher;

public class Robot {

    // SUBSYSTEM DECLARATIONS
    public Component[] components;
    public SampleMecanumDrive drive;
    public Deposit deposit;
    public ArmWrist arm;
    public Intake intake;
    public Slides slides;
    public DroneLauncher drone;
    public Relocalization relocalization;
    public HardwareMap hardwareMap;

    // STATE VARS
    boolean auton;
    Levels subsystemState;


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

                new StepperServo(1, "arm1", map),                     //7
                new StepperServo(1, "arm2", map),                     //8
                new StepperServo(1, "wrist", map),                    //9

                new Motor(0, "intakeMotor", map, false),      //10
                new StepperServo(1, "intakeServo", map),              //11

                new StepperServo(1, "deposit", map)                   //12
        };

        VoltageSensor voltageSensor = map.voltageSensor.iterator().next();

        // INIT SUBSYSTEMS

        this.deposit = new Deposit((StepperServo) components[12]);
        this.arm = new ArmWrist((StepperServo) components[7], (StepperServo) components[8], (StepperServo) components[9]);
        this.intake = new Intake((StepperServo) components[11], (Motor) components[10]);
        this.slides = new Slides((Motor) components[4], (Motor) components[5], (Motor) components[6], voltageSensor);
        this.hardwareMap = map;

        this.subsystemState = Levels.ZERO;
    }

    // INTAKE
    public void intakePreset() {
        this.arm.wristToPos(90); //turning to get through the thingy
        this.slides.runToPreset(Levels.INTAKE);
        this.arm.turnToPreset(Levels.INTAKE);
        this.deposit.open();
        this.subsystemState = Levels.INTAKE;
    }

    public void startIntake() {
        this.intake.startIntake();
    }

    public void startSmartIntake() {
        // add sensor stuff to auto-stop
    }

    public void depositPreset() {
        this.arm.wristToPos(90); //turning to get through the thingy
        this.slides.runToPreset(Levels.DEPOSIT);
        this.arm.turnToPreset(Levels.DEPOSIT);
        this.subsystemState = Levels.DEPOSIT;
    }
    public void runToAutoSpikePreset() {
        this.slides.runToPosition(100);
        this.arm.armToPos(1);
        this.arm.wristToPos(1);
    }

    public void runToAutoBackdropPreset() {
        this.slides.runToPosition(100);
        this.arm.armToPos(1);
        this.arm.wristToPos(1);
    }

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
