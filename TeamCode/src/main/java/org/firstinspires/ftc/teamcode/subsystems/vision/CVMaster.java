package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.relocalization.AprilTagsRelocalization;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

public class CVMaster {
    private VisionPortal visionPortal;
    private YoinkP2Pipeline colourMassDetectionProcessor;
    AprilTagProcessor processor;
    AllianceColor allianceColor;
    HardwareMap hardwareMap;
    AprilTagProcessor tagProcessor;
    AprilTagsRelocalization relocalization;

    public CVMaster(HardwareMap map, AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
        hardwareMap = map;
    }

    public void init() {
        Scalar lower = new Scalar(103, 120, 50);
        Scalar upper = new Scalar(130, 255, 250);

        if (allianceColor == AllianceColor.BLUE) {
            lower = new Scalar(103, 120, 50); // the lower hsv threshold for your detection
            upper = new Scalar(130, 255, 250); // the upper hsv threshold for your detection
        }
        double minArea = 3000; // the minimum area for the detection to consider for your prop

        colourMassDetectionProcessor = new YoinkP2Pipeline(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessors(colourMassDetectionProcessor, tagProcessor)
                .build();

        relocalization = new AprilTagsRelocalization(tagProcessor);
    }

    public void kill() {
        colourMassDetectionProcessor.close();
        visionPortal.close();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }
    }
}
