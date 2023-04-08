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
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.reflect.Field;

@Config
//@Disabled
@TeleOp(name = "Arm Test")
public class ArmTest extends LinearOpMode {
    //Innit Global Variables

    private MotorEx armLeft;
    private MotorEx armRight;
    private MotorEx fold;

    public static int targetA;
    public static int targetF;

    public static double aKp = 0.00332, aKi = 0.00254, aKd = 0.00322, aKf = 0;
    public static double fKp = 0.01, fKi = 0, fKd = 0.0004, fKf = 0;

    PIDFController armController = new PIDFController(aKp, aKi, aKd, aKf);
    PIDFController foldController = new PIDFController(fKp, fKi, fKd, fKf);


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

        waitForStart();

        while(opModeIsActive()) {
            long time = System.currentTimeMillis();


            armController.setP(aKp);
            armController.setI(aKi);
            armController.setD(aKd);
            armController.setF(aKf);
            foldController.setP(fKp);
            foldController.setI(fKi);
            foldController.setD(fKd);
            foldController.setF(fKf);


            int currA = armLeft.getCurrentPosition();
//        if(currA < 1500) {
//            System.out.println("1st " + currA + " | " + targetA);
//            armLeft.set(armController.calculate(currA, Math.max(currA - 50, targetA)));
//            armRight.set(armController.calculate(currA, Math.max(currA - 50, targetA)));
//        }
//        else if(currA > 1500) {
//            System.out.println("2nd " + currA + " | " + targetA);
//            armLeft.set(armController.calculate(currA, Math.min(currA + 50, targetA)));
//            armRight.set(armController.calculate(currA, Math.min(currA + 50, targetA)));
//        }
//        else {
//            System.out.println("3rd " + currA + " | " + targetA);
            armLeft.set(armController.calculate(currA, targetA));
            armRight.set(armController.calculate(currA, targetA));
//        }

            fold.set(foldController.calculate(fold.getCurrentPosition(), targetF));

            telemetry.addData("aLPos ", armLeft.getCurrentPosition());
            telemetry.addData("aRPos ", armRight.getCurrentPosition());
            telemetry.addData("fPos ", fold.getCurrentPosition());
            telemetry.addData("aLVelo", armLeft.getVelocity());
            telemetry.addData("aRVelo", armRight.getVelocity());
            telemetry.addData("fVelo", fold.getVelocity());
            telemetry.addData("aLPow", armLeft.motor.getPower());
            telemetry.addData("aRPow", armRight.motor.getPower());
            telemetry.addData("fPow", fold.motorEx.getPower());
            telemetry.addData("targetA ", targetA);
            telemetry.addData("targetF ", targetF);
            try {
                Field field = armController.getClass().getDeclaredField("totalError");
                field.setAccessible(true);
                telemetry.addData("aError ", (double) field.get(armController));
                telemetry.addData("fError ", (double) field.get(foldController));
            } catch (IllegalAccessException | NoSuchFieldException e) {
                throw new RuntimeException(e);
            }

            telemetry.update();

            System.out.println(System.currentTimeMillis()-time);
            sleep(100-(System.currentTimeMillis()-time));
        }
    }

}