package org.firstinspires.ftc.teamcode.subsystems.relocalization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Relocalization {
    public AprilTagsRelocalization aprilTags;

    public Relocalization(HardwareMap map) {
        aprilTags =  new AprilTagsRelocalization(map.get(WebcamName.class, "Webcam 1"));
    }

    public Pose2d relocalizeUsingBackdrop(Pose2d currentPose) {
        aprilTags.detectBackdrop();
        return aprilTags.getAbsolutePose2d(currentPose);
    }
}
