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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Auton (Wall Left)")
public class WallLeft extends LinearOpMode {
    //Innit Global Variables
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor leftArm;
    private DcMotor rightArm;
    private Servo rightClaw;
    private Servo leftClaw;
    private DcMotor arm_fold;

    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;


    static final double FEET_PER_METER = 3.28084;
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
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        rightClaw = hardwareMap.get(Servo.class, "left-claw");
        leftClaw = hardwareMap.get(Servo.class, "right-claw");
        leftArm = hardwareMap.get(DcMotor.class, "left-arm");
        rightArm = hardwareMap.get(DcMotor.class, "right-arm");
        arm_fold = hardwareMap.get(DcMotor.class, "arm_fold");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        //Motor Setups
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setTargetPosition(0);
        rightArm.setTargetPosition(0);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_fold.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_fold.setTargetPosition(0);
        arm_fold.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
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

                        //For every detection (most likely just one)
                        for (AprilTagDetection tag : currentDetections) {
                            //Check if detection is one of the tags we are looking for
                            if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                                //If detection has one of the tags, set "tagOfInterest" to that tag
                                tagOfInterest = tag;
                                break;
                            }
                        }
                        //Else if a detection in not found
                    } else telemetry.addLine("Don't see any tags :(");

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

        double power = 0.8;

        //Drive Code
        //Grab Preload
        clawChange(true);
        //drive forward to tall pole
        drive(power, 70, 70);
        //Move Arm to Position
        arm_move(760);
        arm_fold_move(655);
        //Don't do anything while arm is moving
        while (arm_fold.isBusy() || isBusy()) {
        }
        //Further positioning to get preload above pole
        drive(power, 12, -12);
        drive(power, 8, 8);
        //Sleep just to prevent any movement
        sleep(200);
        //Release preload
        clawChange(false);
        //Sleep command because servos are delayed
        sleep(1000);
        //Reposition away from pole
        drive(power, -7.8, -7.8);
        drive(power, -12, 12);

        afterQual();

        //Return arm back to normal position to prepare for driver control innit
        arm_move(0);
        arm_fold_move(0);
        //Drive back to middle of parking spots
        drive(power, -30, -30);

        //Signal Conditions
        if (tagOfInterest != null) {
            if (tagOfInterest.id == LEFT) {
                turnLeft();
                drive(power, 36, 36);
            } else if (tagOfInterest.id == RIGHT) {
                turnRight();
                drive(power, 36, 36);
            }
        }

        while (isBusy() || arm_fold.isBusy()) {
            sleep(20);
        }

    }

    // Utility Functions
    private void arm_move(int target) {
        leftArm.setTargetPosition(target);
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
        return leftArm.isBusy() && rightArm.isBusy();
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void drive(double power, double leftInches, double rightInches) {

        int rightTarget;
        int leftTarget;

        if (opModeIsActive()) {
            // Create target positions
            rightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * DRIVE_COUNTS_PER_IN);
            leftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * DRIVE_COUNTS_PER_IN);

            // set target position
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);

            //switch to run to position mode
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //run to position at the designated power
            leftDrive.setPower(power);
            rightDrive.setPower(power);

            // wait until both motors are no longer busy running to position
            while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {}
        }

        // set motor power back to 0
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void turnRight() {
        drive(0.6, 32, -32);
    }

    private void turnLeft() {
        drive(0.6, -33, 33);
    }

    private void clawChange(boolean bool) {
        if (bool) {
            rightClaw.setPosition(-0.4);
            leftClaw.setPosition(0.4);
            return;
        }

        rightClaw.setPosition(0);
        leftClaw.setPosition(-0.4);
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