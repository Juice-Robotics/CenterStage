package org.firstinspires.ftc.teamcode;

// IMPORT SUBSYSTEMS

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
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

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.logging.Level;

public class Robot {

    // SUBSYSTEM DECLARATIONS
    public Component[] components;
    public SampleMecanumDriveCancelable drive;
    public Claw claw;
    public ArmElbow arm;
    public Intake intake;
    public IntakeSensor intakeSensor;
    public Slides slides;
    public DroneLauncher drone;
    public Relocalization relocalization;
    public HardwareMap hardwareMap;

    // STATE VARS
    boolean auton;
    public Levels subsystemState = Levels.INTAKE;
    public boolean intaking = false;
    public ArrayList<RobotFlags> flags = new ArrayList<>();
    public double CURRENT_HIGH = 1;
    public double ENCODER_MAX_DIFFERENCE = 1;
    public ElapsedTime antiJamCooldown = new ElapsedTime();
    public boolean threadState = false;

    Motor backLeft;
    Motor backRight;
    Motor frontLeft;
    Motor frontRight;


    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        this.drive = new SampleMecanumDriveCancelable(map);

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
        this.intakeSensor = new IntakeSensor(map.colorSensor.get("intakeSensor1"), map.colorSensor.get("intakeSensor2"));
        this.slides = new Slides((Motor) components[4], (Motor) components[5], (Motor) components[6], (StepperServo) components[7], voltageSensor);
        this.drone = new DroneLauncher((StepperServo) components[16]);
        this.relocalization = new Relocalization();
        this.hardwareMap = map;

        this.subsystemState = Levels.ZERO;

        backLeft = (Motor) components[0];
        backRight = (Motor) components[1];
        frontLeft = (Motor) components[2];
        frontRight = (Motor) components[3];
    }

    // INTAKE
    public void intakePreset() {
        //turning to get through the thingy
        this.slides.runToPreset(Levels.INTAKE);
        this.arm.runtoPreset(Levels.INTAKE);
        this.claw.setClawOpen();
        this.claw.runToWristPreset(Levels.INTAKE);
        this.intake.runToPreset(Levels.INTAKE);
        this.subsystemState = Levels.INTAKE;
    }

    public void startIntake() {
        intaking = true;
        this.intake.startIntake();
        this.arm.runtoPreset(Levels.INTAKE);
        this.claw.setClawOpen();
        this.intake.runToPreset(Levels.INTAKE);
        this.claw.runToWristPreset(Levels.INTAKE);
        this.slides.runToPosition(0);
    }

    public void startAutoIntake() {
        intaking = true;
        this.intake.startIntake();
        this.arm.runtoPreset(Levels.INTAKE);
        this.intake.runToPreset(Levels.INTAKE);
        this.claw.runToWristPreset(Levels.INTAKE);
        this.slides.runToPosition(0);
    }

    public void stopIntake() {
        intaking = false;
        this.arm.runtoPreset(Levels.CAPTURE);
        Thread thread = new Thread(new Runnable() {
            public void run() {
                sleep(150);
                claw.setClawClose();
                sleep(350);
                intake.stopIntake();
                intake.runToPreset(Levels.INTERMEDIATE);
                arm.runtoPreset(Levels.INTERMEDIATE);
            }});
        thread.start();
        subsystemState = Levels.INTERMEDIATE;
    }

    public void autoIntake() {
        intaking = true;
        this.intake.reverseIntake();
        this.arm.runtoPreset(Levels.INTAKE);
        this.claw.setClawOpen();
        this.intake.runToPreset(Levels.INTAKE);
        this.claw.runToWristPreset(Levels.INTAKE);
        this.slides.runToPosition(0);
    }

    public void initPos() {
        intaking = false;
        this.claw.runToWristPreset(Levels.DEPOSIT);
        this.intake.runToPreset(Levels.INIT);
        sleep(500);
        this.arm.runtoPreset(Levels.INIT);
        sleep(2000);
        this.claw.setClawClose();
//        this.intake.setAngle(50);
        this.subsystemState = Levels.INIT;
    }

    public void farPos() {
        intaking = false;
        this.claw.runToWristPreset(Levels.DEPOSIT);
        this.intake.runToPreset(Levels.INIT);
        this.arm.setAngleArm(140);
        this.arm.setAngleElbow(211);
        this.subsystemState = Levels.FARPOS;
    }

    /**
     * <h1>WARNING: BLOCKS THREAD</h1>
     * Blocks thread until intake is complete with specified number of pixels (1-2)
     */
    public void startSmartIntake(int pixels) {
        this.startIntake();
        if (pixels == 1) {
            while (!intakeSensor.hasPixel()[0] && !intakeSensor.hasPixel()[1]) {
                sleep(20);
            }
        } else if (pixels == 2) {
            while (!intakeSensor.hasPixel()[0] || !intakeSensor.hasPixel()[1]) {
                sleep(20);
            }
        }
        this.stopIntake();
    }

    public void smartClawOpen() {
        this.claw.setClawOpen();
        Thread thread = new Thread(new Runnable() {
            public void run() {
                sleep(300);
                arm.setAngleArm(30);
                arm.setAngleElbow(106);
                claw.runToWristPreset(Levels.INTAKE);
                sleep(300);
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

    public void depositPreset() {
        this.slides.runToPreset(Levels.DEPOSIT);
        Thread thread = new Thread(new Runnable() {
            public void run() {
                sleep(700);
                arm.runtoPreset(Levels.DEPOSIT);
                claw.runToWristPreset(Levels.DEPOSIT);
            }});
        thread.start();
        this.subsystemState = Levels.DEPOSIT;
    }

    public void autoPreloadDepositPreset() {
        this.slides.runToPosition(200);
        Thread thread = new Thread(new Runnable() {
            public void run() {
                sleep(300);
                arm.runtoPreset(Levels.DEPOSIT);
                claw.runToWristPreset(Levels.DEPOSIT);
            }});
        thread.start();
        this.subsystemState = Levels.DEPOSIT;
    }

    public void autoCycleDepositPreset() {
        this.slides.runToPosition(300);
        Thread thread = new Thread(new Runnable() {
            public void run() {
                sleep(300);
                arm.runtoPreset(Levels.DEPOSIT);
                claw.runToWristPreset(Levels.DEPOSIT);
            }});
        thread.start();
        this.subsystemState = Levels.DEPOSIT;
    }

    public void runToAutoSpikePreset() {
        this.slides.runToPosition(100);
        this.arm.runtoPreset(Levels.DEPOSIT);
    }

    public void runToAutoBackdropPreset() {
        this.slides.runToPosition(100);
        this.arm.runtoPreset(Levels.BACKDROP);
    }

    public void climbExtend() {
//        this.flags.add(RobotFlags.CLIMB_ENGAGED);
//        this.flags.add(RobotFlags.CLIMB_EXTEND_IN_PROGRESS);
        this.slides.runToClimb();
        this.intake.runToPreset(Levels.CLIMB_EXTEND);
        Thread thread = new Thread(new Runnable() {
            public void run() {
                sleep(600);
                arm.runtoPreset(Levels.DEPOSIT);
                claw.runToWristPreset(Levels.DEPOSIT);
                while (slides.getPos() <= 460) {
                    // sleep
                }
                    slides.setPower((float) 0.6);
                    slides.shiftGear(true);
                    sleep(100);
                    slides.setPower(0);

//                if (flags.contains(RobotFlags.CLIMB_RETRACT_REQUESTED)) {
//                    slides.shiftGear(false);
//                    sleep(100);
//                    smartClawOpen();
//                    flags.remove(RobotFlags.CLIMB_RETRACT_REQUESTED);
//                }
//                flags.remove(RobotFlags.CLIMB_EXTEND_IN_PROGRESS);
            }});
        thread.start();
    }

    /**
     * <h1>Climb Disengage, NOT Climb Start</h1>
     * Use <code>startClimb()</code> in order to climb.
     */
    public void climbRetract() {
        this.flags.remove(RobotFlags.CLIMB_ENGAGED);
        if (flags.contains(RobotFlags.CLIMB_EXTEND_IN_PROGRESS)) {
            flags.add(RobotFlags.CLIMB_RETRACT_REQUESTED);
        } else {
            slides.shiftGear(false);
            sleep(100);
            smartClawOpen();
        }
    }

    public void antiJam(){
        if (intaking) {
            if (this.intake.intakeMotor.getCurrent() > 5 && !flags.contains(RobotFlags.ANTI_JAM_IN_PROGRESS) && antiJamCooldown.time(TimeUnit.MILLISECONDS) >= 250) {
                flags.add(RobotFlags.INTAKE_JAMMED);
                flags.add(RobotFlags.ANTI_JAM_IN_PROGRESS);
                this.intake.setAngle(100);
                this.intake.reverse();
                sleep(250);
                this.intake.runToPreset(Levels.INTAKE);
                this.intake.intakeMotor.setSpeed(1);
                flags.remove(RobotFlags.INTAKE_JAMMED);
                flags.remove(RobotFlags.ANTI_JAM_IN_PROGRESS);
                antiJamCooldown.reset();
            }
        }
    }


    public void startClimb() {
        this.slides.startClimb();
        this.subsystemState = Levels.CLIMB;
    }

    public void launchSubsystemThread(Telemetry telemetry) {
        threadState = true;
        telemetry.addData("Subsys Threads State:", "STARTING");
        telemetry.update();
        Thread t1 = new Thread(() -> {
            telemetry.addData("Subsys Threads State:", "STARTED");
            telemetry.update();
            while (threadState == true) {
                slides.update();
                antiJam();
            }
            telemetry.addData("Subsys Threads State:", "STOPPED");
            telemetry.update();
        });
        t1.start();
    }

    public void destroyThreads(Telemetry telemetry) {
        telemetry.addData("Slides Threads State:", "STOPPING");
        telemetry.update();
        threadState = false;
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

        frontLeft.setSpeed((float)powerFrontLeft);
        frontRight.setSpeed((float)powerFrontRight);
        backLeft.setSpeed(-(float)powerBackLeft);
        backRight.setSpeed(-(float)powerBackRight);
    }

    public void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}