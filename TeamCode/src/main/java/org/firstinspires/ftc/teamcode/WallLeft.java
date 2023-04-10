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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.rr.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Auton Left")
public class WallLeft extends LinearOpMode {
    //Innit Global Variables
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private DcMotor top_left_drive;
    private DcMotor top_right_drive;
    private DcMotor bottom_left_drive;
    private DcMotor bottom_right_drive;
    private DcMotor leftArm;
    private DcMotor rightArm;
    private Servo rightClaw;
    private Servo leftClaw;
    private DcMotor arm_fold;
    private SampleMecanumDrive MecDrive;
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
        top_left_drive = hardwareMap.get(DcMotor.class, "top_left_drive");
        top_right_drive = hardwareMap.get(DcMotor.class, "top_right_drive");
        bottom_left_drive = hardwareMap.get(DcMotor.class, "bottom_left_drive");
        bottom_right_drive = hardwareMap.get(DcMotor.class, "bottom_right_drive");
        rightClaw = hardwareMap.get(Servo.class, "right-claw");
        leftClaw = hardwareMap.get(Servo.class, "left-claw");
        leftArm = hardwareMap.get(DcMotor.class, "left-arm");
        rightArm = hardwareMap.get(DcMotor.class, "right-arm");
        arm_fold = hardwareMap.get(DcMotor.class, "arm_fold");

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        //Road Runner Innit
        MecDrive = new SampleMecanumDrive(hardwareMap);



        telemetry.addLine("Initialization Complete");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            sleep(20);
        }

        runtime.reset();

        //Detection
        boolean detectionPassed = false;

        while (!detectionPassed) {
            try {
                while (runtime.seconds() < 3 && tagOfInterest == null) {
                    //Put Detections into ArrayList
                    ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                    //If a detection is found
                    if (!currentDetections.isEmpty()) {
                        telemetry.addLine("It is detecting something, just not any of our tags");
                        //For every detection (most likely just one)
                        for (AprilTagDetection tag : currentDetections) {
                            //Check if detection is one of the tags we are looking for
                            if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                                //If detection has one of the tags, set "tagOfInterest" to that tag
                                tagOfInterest = tag;
                                break;
                            }
                        }
                        //else if a detection in not found
                    } else telemetry.addLine("Don't see any tags :(");

                    telemetry.addData("Time Elapsed", runtime.milliseconds());
                    telemetry.update();
                    sleep(20);
                }

                camera.closeCameraDevice();

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
                detectionPassed = true;

            } catch (Throwable ignored) {
                telemetry.addLine("Tag crashed, let's try again!");
            }

            telemetry.update();
            sleep(20);
        }

        //Drive Functions

    }

    // Utility Functions
    private void arm_move(int target)
    {
        leftArm.setTargetPosition(-target);
        rightArm.setTargetPosition(target);
        leftArm.setPower(1);
        rightArm.setPower(1);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void arm_fold_move(int target) {
        arm_fold.setTargetPosition(target);
        arm_fold.setPower(1);
        arm_fold.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private boolean isBusy() {
        if(leftArm.isBusy() && !rightArm.isBusy()) return true;
        if(!leftArm.isBusy() && rightArm.isBusy()) return true;
        if(!leftArm.isBusy() && !rightArm.isBusy()) return true;
        return false;
    }

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