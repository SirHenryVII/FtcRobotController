package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Camera {
    public final OpenCvCamera camera;
    private final OpMode opMode;
    private final AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public AprilTagDetection tagOfInterest = null;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag IDs 1,2, and 3 from the 36h11 family

    final int LEFT = 1;
    final int MIDDLE = 2;
    final int RIGHT = 3;

    public Camera(OpMode opMode) {
        this.opMode = opMode;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                opMode.telemetry.addData("ERROR", "Camera initialization error: " + errorCode);
            }
        });
    }

    public void search() {
        try {
            //Put Detections into ArrayList
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            //If a detection is found
            if (!currentDetections.isEmpty()) {
                //For every detection (most likely just one)
                for (AprilTagDetection tag : currentDetections) {
                    //Check if detection is one of the tags we are looking for
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        //If detection has one of the tags, set "tagOfInterest" to that tag
                        tagOfInterest = tag;
                        break;
                    }
                }
            }

            if(tagOfInterest!=null) opMode.telemetry.addData("CurrentDetection: ", tagOfInterest.id);
            else opMode.telemetry.addData("CurrentDetection: ", "null");
            opMode.telemetry.update();

        } catch (Throwable ignored) {
            ignored.printStackTrace();
            opMode.telemetry.addLine("Tag crashed, let's try again!");
        }

        opMode.telemetry.update();
    }
}
