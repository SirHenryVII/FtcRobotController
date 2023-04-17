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

package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.arm.ArmHandler;
import org.openftc.apriltag.AprilTagDetection;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "Auton Left High")
public class AutoLeftHigh extends LinearOpMode {
    public enum AutoCones {
        FIVE(280, 960),
        FOUR(375, 975),
        THREE(330, 940),
        TWO(300, 940),
        ONE(240, 900);

        final int arm;
        final int fold;

        AutoCones(int arm, int fold) {
            this.arm = arm;
            this.fold = fold;
        }
    }
    AutoCones[] autoCones = new AutoCones[] {
            AutoCones.ONE,
            AutoCones.TWO,
            AutoCones.THREE,
            AutoCones.FOUR,
            AutoCones.FIVE
    };
    //Innit Global Variables
    public Camera camera;

    private Servo rightClaw;
    private Servo leftClaw;

    ArmHandler armHandler;
    VAPDController driveController;
    private final AtomicBoolean threadingEnabled = new AtomicBoolean(true);

    @Override
    public void runOpMode() {
        //April Tags Stuff

        camera = new Camera(this);

        telemetry.setMsTransmissionInterval(50);

        //Innit Motors
        driveController = new VAPDController(
                new MotorEx(hardwareMap, "top_left_drive", 28 * (5.23 * 3.61), 6000 / (5.23 * 3.61)),
                new MotorEx(hardwareMap, "top_right_drive", 28 * (5.23 * 3.61), 6000 / (5.23 * 3.61)),
                new MotorEx(hardwareMap, "bottom_left_drive", 28 * (5.23 * 3.61), 6000 / (5.23 * 3.61)),
                new MotorEx(hardwareMap, "bottom_right_drive", 28 * (5.23 * 3.61), 6000 / (5.23 * 3.61)));
        leftClaw = hardwareMap.get(Servo.class, "left-claw");
        rightClaw = hardwareMap.get(Servo.class, "right-claw");
        driveController.scalerMap.add(0, 0.1);
        driveController.scalerMap.add(.25, 1);
        driveController.scalerMap.add(.75, 1);
        driveController.scalerMap.add(1.0, 0.1);

        armHandler = new ArmHandler(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        clawChange(false);

        while (!isStarted() && !isStopRequested()) {
            camera.search();
            sleep(20);
        }
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis()-start < 5000) {
            camera.search();
            sleep(20);
        }
        camera.camera.closeCameraDevice();

        double parkMult = 0;
        //add parking telemetry
        if (camera.tagOfInterest != null) {
            switch (camera.tagOfInterest.id) {
                case 1:
                    parkMult = 1.15;
                    telemetry.addLine("Parking: Left (1)");
                    break;
                case 2:
                    telemetry.addLine("Parking: Middle (2)");
                    break;
                case 3:
                    parkMult = -1.15;
                    telemetry.addLine("Parking: Right (3)");
                    break;
            }
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(camera.tagOfInterest);
        } else {
            telemetry.addLine("Tag was not sighted :(");
            telemetry.addLine("Parking: Middle (2) (Default)");
            telemetry.update();
        }


        //start arm thread
        new Thread(new ArmThread(threadingEnabled, armHandler)).start();


        //Drive Functions
        straight(1.275);
        strafe(-1.2);
        armHandler.setPosition(1000, 0);
        driveController.spin(620);
        sleep(1000);
        while(armHandler.isBusy()) {}
        armHandler.setPosition(900, ArmHandler.State.HIGH.fold);
        sleep(1000);
        while(armHandler.isBusy()) {}
        clawChange(true);
        sleep(2000);
        driveController.spin(-620);
        while(driveController.isBusy()) {}
        strafe(1.3);
        strafe(parkMult);
        armHandler.setState(ArmHandler.State.START);
        sleep(1000);
        while(armHandler.isBusy()) {}
//        while(driveController.isBusy()) {}
//        armHandler.setState(ArmHandler.State.START);
//        straight(0.2);
//        while(driveController.isBusy()) {}
//        driveController.strafe((int) (1300*parkMult));
//        while(driveController.isBusy()) {}
//        straight(1.35);
//        driveController.spin(-1250);
//        while(driveController.isBusy()) {}
//        straight(0.443);
//        while(driveController.isBusy()) {}
//        armHandler.setPosition(AutoCones.FIVE.arm, AutoCones.FIVE.fold);
//        sleep(2000);
//        while(armHandler.isBusy()) {}
//        clawChange(false);
//        sleep(2000);
//        armHandler.setPosition(AutoCones.FIVE.arm, 600);
//        sleep(1000);
//        while(armHandler.isBusy()) {}
//        armHandler.setPosition(1000, 1400);
//        sleep(2000);
//        straight(-1.654);
//        while(driveController.isBusy()) {}
//        sleep(1000);
//        driveController.spin(-480);
//        sleep(1000);
//        while(driveController.isBusy()) {}
//        straight(0.1);
//        clawChange(true);
//        sleep(2000);
//        straight(-0.1);
//        armHandler.setState(ArmHandler.State.START);
//        driveController.spin(480);
//        sleep(2000);
//        while(driveController.isBusy()) {}
//        straight(1.2);
//        driveController.strafe(1400);
//        sleep(2000);
//        while(driveController.isBusy()) {}
//        armHandler.setState(ArmHandler.State.START);
//        driveController.spin(1250);
//        straight(-2.5);
//        driveController.strafe(1500);
//        while(armHandler.isBusy()) {}
//        while(driveController.isBusy()) {}



        threadingEnabled.set(false);
    }

    // Utility Functions

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void straight(double mult) {
        driveController.straight((int) (mult*1200));
        sleep(1000);
        while(driveController.isBusy()) {}
    }

    public void strafe(double mult) {
        driveController.strafe((int) (mult*1200));
        sleep(1000);
        while(driveController.isBusy()) {}
    }

    private void clawChange(boolean open) {
        if (open) {
            rightClaw.setPosition(1);
            leftClaw.setPosition(0);
            return;
        }
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.3);
    }
}