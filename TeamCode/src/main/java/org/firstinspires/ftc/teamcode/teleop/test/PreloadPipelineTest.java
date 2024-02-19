package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.Motor;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

/**
 * This 2023-2024 OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp
//@Disabled
public class PreloadPipelineTest extends LinearOpMode {
    Motor backLeft;
    Motor backRight;
    Motor frontLeft;
    Motor frontRight;

    @Override
    public void runOpMode() {
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        CVMaster cv = new CVMaster(hardwareMap);
        cv.initPreload();
        backLeft = new Motor(3, "leftRear", hardwareMap, true);       //0 left odometer
        backRight = new Motor(2, "rightRear", hardwareMap, false);        //1 right odometer
        frontLeft = new Motor(1, "leftFront", hardwareMap, true);         //2 middle odometer
        frontRight = new Motor(0, "rightFront", hardwareMap, false);
        double x;
        double y;
        double rx;

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("preload", cv.preloadProcessor.getPreloadedZone());
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.right_trigger > 0.5) {
                    x = -gamepad1.left_stick_x * (1 - 0.66 * gamepad1.right_trigger);
                    y = -gamepad1.left_stick_y * (1 - 0.66 * gamepad1.right_trigger);
                    rx = gamepad1.right_stick_x * (1 - 0.66 * gamepad1.right_trigger);

                } else {
                    x = -gamepad1.left_stick_x;
                    y = -gamepad1.left_stick_y;
                    rx = gamepad1.right_stick_x;
                }
                setDrivePower(-x, y, rx);
                telemetry.update();
            }
        }
    }

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
}