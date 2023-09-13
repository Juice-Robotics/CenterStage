package org.firstinspires.ftc.teamcode;

// IMPORT SUBSYSTEMS

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.Component;
import org.firstinspires.ftc.teamcode.lib.Motor;

public class Robot {

    // SUBSYSTEM DECLARATIONS
    public Component[] components;
    public SampleMecanumDrive drive;
    boolean auton;

    // STATE VARS



    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        this.drive = new SampleMecanumDrive(map);

//        this.cv = new CVMaster(map);

        this.components = new Component[]{
                new Motor(3, "leftRear", map, true),          //0 left odometer
                new Motor(2, "rightRear", map, false),        //1 right odometer
                new Motor(1, "leftFront", map, true),         //2 middle odometer
                new Motor(0, "rightFront", map, false),       //3
        };

        // INIT SUBSYSTEMS



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
