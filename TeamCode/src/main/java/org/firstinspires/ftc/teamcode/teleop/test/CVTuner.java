package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.StepperServo;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Claw;
import org.firstinspires.ftc.teamcode.subsystems.vision.YoinkP2Pipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@TeleOp(group = "competition")
@Config
public class CVTuner extends LinearOpMode {
    public static double minArea = 3000; // the minimum area for the detection to consider for your prop
    public static double lowerH = 103;
    public static double lowerS = 120;
    public static double lowerV = 50;

    public static double upperH = 130;
    public static double upperS = 255;
    public static double upperV = 250;
    Scalar lower = new Scalar(lowerH, lowerS, lowerV); // the lower hsv threshold for your detection
    Scalar upper = new Scalar(upperH, upperS, upperV); // the upper hsv threshold for your detection

    public static double LEFT_BOUNDARY = 213;
    public static double RIGHT_BOUNDARY = 426;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
        // the left dividing line, in this case the left third of the frame
        // the left dividing line, in this case the right third of the frame
        YoinkP2Pipeline colourMassDetectionProcessor = new YoinkP2Pipeline(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> LEFT_BOUNDARY, // the left dividing line, in this case the left third of the frame
                () -> RIGHT_BOUNDARY // the left dividing line, in this case the right third of the frame
        );
        // the camera on your robot is named "Webcam 1" by default
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(colourMassDetectionProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(colourMassDetectionProcessor, 0);

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            Scalar lower = new Scalar(lowerH, lowerS, lowerV); // the lower hsv threshold for your detection
            Scalar upper = new Scalar(upperH, upperS, upperV); // the upper hsv threshold for your detection
            colourMassDetectionProcessor.changeConstants(
                    lower,
                    upper,
                    () -> minArea,
                    () -> LEFT_BOUNDARY,
                    () -> RIGHT_BOUNDARY
            );



        }
    }
}