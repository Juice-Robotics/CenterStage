package org.firstinspires.ftc.teamcode.auton.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.vision.TeamElementCVProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.YoinkElementCVProcessor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(group = "drive", name = "cvtest")

public class cvtestig extends LinearOpMode {
    Robot robot;
    VisionPortal visionPortal;
    YoinkElementCVProcessor teamElementProcessor;
    YoinkElementCVProcessor.PropLocation propLocation = YoinkElementCVProcessor.PropLocation.UNFOUND;

    @Override
    public void runOpMode() throws InterruptedException {


        robot = new Robot(hardwareMap, true);
        teamElementProcessor = new YoinkElementCVProcessor(AllianceColor.RED);
        teamElementProcessor.alliance = AllianceColor.RED;
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .addProcessor(teamElementProcessor)

                .build();

        FtcDashboard.getInstance().startCameraStream(teamElementProcessor, 30);


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            YoinkElementCVProcessor.PropLocation reading = teamElementProcessor.getLocation();
            telemetry.addData("Camera State", visionPortal.getCameraState());

            if (reading == YoinkElementCVProcessor.PropLocation.UNFOUND) {
                telemetry.addLine("Team Element Location: <b>NOT FOUND</b>");
            } else {
                telemetry.addData("Team Element Location", reading);
            }

            telemetry.update();
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        // shuts down the camera once the match starts, we dont need to look any more

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        propLocation = teamElementProcessor.getLocation();

//        // if it is UNFOUND, you can manually set it to any of the other positions to guess
//        if (propLocation == TeamElementCVProcessor.Location.UNFOUND) {
//            propLocation = TeamElementCVProcessor.Location.CENTER;
//        }


        waitForStart();

        if (isStopRequested()) return;


        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}