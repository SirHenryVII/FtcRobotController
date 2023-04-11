/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.arm.ArmHandler;
import org.firstinspires.ftc.teamcode.drive.ArmThread;
import org.firstinspires.ftc.teamcode.drive.RunThread;
import org.firstinspires.ftc.teamcode.drive.VAPDController;
import org.firstinspires.ftc.teamcode.rr.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "Auton Left")
public class WallLeft extends LinearOpMode {
    //Innit Global Variables
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private Servo rightClaw;
    private Servo leftClaw;
    ElapsedTime runtime = new ElapsedTime();

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

    AprilTagDetection tagOfInterest = null;

    ArmHandler armHandler;
    VAPDController driveController;
    private final AtomicBoolean threadingEnabled = new AtomicBoolean(true);

    @Override
    public void runOpMode() {
        //April Tags Stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("ERROR", "Function \"onError\" Triggered on Line 78");
            }
        });

        telemetry.setMsTransmissionInterval(50);

        //Innit Motors
        driveController = new VAPDController(
                hardwareMap.get(DcMotor.class, "top_left_drive"),
                hardwareMap.get(DcMotor.class, "top_right_drive"),
                hardwareMap.get(DcMotor.class, "bottom_left_drive"),
                hardwareMap.get(DcMotor.class, "bottom_right_drive"));
        driveController.scalerMap.add(0, 0.1);
        driveController.scalerMap.add(.25, 1);
        driveController.scalerMap.add(.75, 1);
        driveController.scalerMap.add(1.0, 0.1);

        armHandler = new ArmHandler(hardwareMap);

        //Detection

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStarted() && !isStopRequested()) {
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

                telemetry.addData("CurrentDetection: ", tagOfInterest.id);
                telemetry.addData("Time Elapsed", runtime.milliseconds());
                telemetry.update();
                sleep(20);

            } catch (Throwable ignored) {
                telemetry.addLine("Tag crashed, let's try again!");
            }

            telemetry.update();
            sleep(20);
        }

        runtime.reset();

        camera.closeCameraDevice();

        //add parking telemetry
        if (tagOfInterest != null) {
            switch (tagOfInterest.id) {
                case LEFT:
                    telemetry.addLine("Parking: Left (1)");
                case MIDDLE:
                    telemetry.addLine("Parking: Middle (2)");
                case RIGHT:
                    telemetry.addLine("Parking: Right (3)");
            }
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
        } else {
            telemetry.addLine("Tag was not sighted :(");
            telemetry.addLine("Parking: Middle (2) (Default)");
        }

        telemetry.update();


        //start arm/drive threads
        new Thread(new RunThread(threadingEnabled, driveController)).start();
        new Thread(new ArmThread(threadingEnabled, armHandler)).start();



        //Drive Functions
        clawChange(false);


    }

    // Utility Functions

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void clawChange(boolean bool) {
        if (bool) {
            leftClaw.setDirection(Servo.Direction.FORWARD);
            rightClaw.setDirection(Servo.Direction.REVERSE);
            rightClaw.setPosition(0.5);
            leftClaw.setPosition(-0.7);
            return;
        }
        leftClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.setPosition(0.3);
        rightClaw.setPosition(0);
    }

    public void afterQual() {
                /*
        //Pick up another cone
        //Lower Arm
        arm_move(350);
        arm_fold_move(200);
        //Drive a little forward
        drive(power, 15, 15);
        //Face next cone
        turnLeft();
        //Drive to cone
        drive(power, 30, 30);
        //Close Claw
        clawChange(true);
        sleep(1000);
        //Raise Arm
        arm_move(750);
        arm_fold_move(655);
        while(arm_fold.isBusy() || arm.isBusy()){}
        //Turn around
        drive(power, -64, 64);
        //Drive to Pole
        drive(power, 25, 25);
        //Drop Cone
        clawChange(false);
        sleep(200);
        //Turn twoards parking
        turnRight();
        */
    }

}