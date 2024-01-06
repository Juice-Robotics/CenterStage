package org.firstinspires.ftc.teamcode.auton.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.vision.TeamElementCVProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled // remove this line to have this show up on your robot
@Autonomous
public class CVTest extends OpMode {
    private VisionPortal visionPortal;
    private TeamElementCVProcessor teamElementProcessor;


    @Override
    public void init() {
        teamElementProcessor = new TeamElementCVProcessor(
                () -> 100, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426, // the left dividing line, in this case the right third of the frame,
                telemetry,
                AllianceColor.BLUE);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(teamElementProcessor)
                .build();
    }


    @Override
    public void init_loop() {
        telemetry.addData("Camera State", visionPortal.getCameraState());
    }


    @Override
    public void start() {
        // shuts down the camera once the match starts, we dont need to look any more
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        // gets the recorded prop position
        TeamElementCVProcessor.Location recordedPropPosition = teamElementProcessor.getLocation();

        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == TeamElementCVProcessor.Location.UNFOUND) {
            recordedPropPosition = TeamElementCVProcessor.Location.CENTER;
        }


        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (recordedPropPosition) {
            case LEFT:
                telemetry.addData("running", "left spike auton");
                break;
            case UNFOUND: // we can also just add the unfound case here to do fallthrough intstead of the overriding method above, whatever you prefer!
                telemetry.addLine("warning: <b>unfound spike auton</b>");
                break;
            case CENTER:
                telemetry.addData("running", "center spike auton");
                break;
            case RIGHT:
                telemetry.addData("running", "right spike auton");
                break;
        }
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        // this closes down the portal when we stop the code, its good practice!
        teamElementProcessor.close();
        visionPortal.close();
    }
}
