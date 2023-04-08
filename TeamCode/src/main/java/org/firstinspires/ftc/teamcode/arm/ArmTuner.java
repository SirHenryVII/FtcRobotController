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

package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.reflect.Field;

@Config
//@Disabled
@TeleOp(name = "Arm Tuner")
public class ArmTuner extends LinearOpMode {
    //Innit Global Variables

    private MotorEx armLeft;
    private MotorEx armRight;
    private MotorEx fold;

    public static boolean velocity = false;
    public static int tunerTarget = 0;
    public static int tunerCycles = 50;

    public PidTuner tuner = new PidTuner();


    @Override
    public void runOpMode() throws InterruptedException {
        armLeft = new MotorEx(hardwareMap, "arm-left",28/(5.23 * 3.61 * 3.61 * 125/72), 6000);
        armRight = new MotorEx(hardwareMap, "arm-right",28/(5.23 * 3.61 * 3.61 * 125/72), 6000);
        fold = new MotorEx(hardwareMap, "arm_fold", 28/(5.23 * 3.61), 6000);

        armLeft.setInverted(true);
        fold.setInverted(true);

        armLeft.resetEncoder();
        armRight.resetEncoder();
        fold.resetEncoder();

        armLeft.setRunMode(Motor.RunMode.RawPower);
        armRight.setRunMode(Motor.RunMode.RawPower);
        fold.setRunMode(Motor.RunMode.RawPower);

        armLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fold.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        tuner.setLoopInterval(100_000);
        tuner.setOutputRange(-0.5f,0.5f);
        tuner.setZNMode(PidTuner.ZNMode.ZNModeNoOvershoot);

        telemetry.addData("aLPos ", armLeft.getCurrentPosition());
        telemetry.addData("aRPos ", armRight.getCurrentPosition());
        telemetry.addData("fPos ", fold.getCurrentPosition());
        telemetry.addData("aLVelo", armLeft.getVelocity());
        telemetry.addData("aRVelo", armRight.getVelocity());
        telemetry.addData("fVelo", fold.getVelocity());
        telemetry.addData("aLPow", armLeft.motor.getPower());
        telemetry.addData("aRPow", armRight.motor.getPower());
        telemetry.addData("fPow", fold.motorEx.getPower());
        telemetry.update();

        waitForStart();

        tuner.setTargetInputValue(tunerTarget);
        tuner.setTuningCycles(tunerCycles);
        tuner.startTuningLoop(System.currentTimeMillis()*1000);

        long microseconds = 0;
        while(!tuner.isFinished() && opModeIsActive()) {
            long time = System.currentTimeMillis();
            long prevMicroseconds = microseconds;
            microseconds = System.currentTimeMillis()*1000;

            float input;
            if(velocity) input = (float) armLeft.getVelocity();
            else input = armLeft.getCurrentPosition();

            double output = tuner.tunePID(input, microseconds);

            armLeft.set(output);
            armRight.set(output);

            telemetry.addData("aLPos ", armLeft.getCurrentPosition());
            telemetry.addData("aRPos ", armRight.getCurrentPosition());
            telemetry.addData("fPos ", fold.getCurrentPosition());
            telemetry.addData("aLVelo", armLeft.getVelocity());
            telemetry.addData("aRVelo", armRight.getVelocity());
            telemetry.addData("fVelo", fold.getVelocity());
            telemetry.addData("aLPow", armLeft.motor.getPower());
            telemetry.addData("aRPow", armRight.motor.getPower());
            telemetry.addData("fPow", fold.motorEx.getPower());
            telemetry.update();

            sleep(100-(System.currentTimeMillis()-time));
        }

        armLeft.set(0);
        armRight.set(0);

        System.out.println(String.format("P: %.5f I: %.5f D: %.5f", tuner.getKp(), tuner.getKi(), tuner.getKd()));
    }

}