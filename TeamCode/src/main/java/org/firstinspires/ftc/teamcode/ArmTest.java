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
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@TeleOp(name = "Arm Test")
public class ArmTest extends OpMode {
    //Innit Global Variables

    private DcMotorEx Arm;
    private DcMotorEx Arm_Fold;

    public static double pA = 0, iA = 0, dA = 0, fA = 0;
    public static double pF = 0, iF = 0, dF = 0, fF = 0;

    private final double ticks_in_degree = (28 * 18.9) / 360;

    public static int targetA;
    public static int targetF;



    private PIDController controllerA;
    private PIDController controllerF;


    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void init() {


        //Innit Motors
        Arm = hardwareMap.get(DcMotorEx.class, "arm");
        Arm_Fold = hardwareMap.get(DcMotorEx.class, "arm_fold");
        Arm.setDirection(DcMotorEx.Direction.REVERSE);
        Arm_Fold.setDirection(DcMotorEx.Direction.FORWARD);


        //innit PID
        controllerA = new PIDController(pA, iA, dA);
        controllerF = new PIDController(pF, iF, dF);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initialization Complete");
        telemetry.update();

    }

    @Override
    public void loop() {
        runtime.reset();

        //PID Loop
        controllerA.setPID(pA, iA, dA);
        controllerF.setPID(pF, iF, dF);

        int armPos = Arm.getCurrentPosition();
        int armFoldPos = Arm_Fold.getCurrentPosition();

        double pidA = controllerA.calculate(armPos, targetA);
        double pidF = controllerF.calculate(armFoldPos, targetF);

        double ffA = Math.cos(Math.toRadians(targetA / ticks_in_degree)) * fA;
        double ffF = Math.cos(Math.toRadians(targetF / ticks_in_degree)) * fF;

        Arm.setPower(pidA + ffA);
        Arm_Fold.setPower(pidF + ffF);

        telemetry.addData("posA ", armPos);
        telemetry.addData("posF ", armFoldPos);
        telemetry.addData("targetA ", targetA);
        telemetry.addData("targetF ", targetF);
        telemetry.update();


    }



}