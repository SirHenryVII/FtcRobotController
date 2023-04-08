package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.reflect.Field;

@Config
@TeleOp(name = "VeloController")
public class VeloController extends LinearOpMode {
    //Innit Global Variables

    private MotorEx armLeft;
    private MotorEx armRight;
    private MotorEx fold;

    public static int armPos;
    public static int foldPos;

    public static double aKp = 0.00332, aKi = 0.00254, aKd = 0.00322, aKf = 0, aKs = 0, aKv = 1, aKa = 0;
    public static double maxVelo = 2500, minVelo = -1000, maxTErr = 1500, aKvp = 3.5, aKvi = 0.4;
    public VEPOSController armController;
    public VEPOSController foldController;


    @Override
    public void runOpMode() throws InterruptedException {
        armLeft = new MotorEx(hardwareMap, "arm-left",28*(5.23 * 3.61 * 3.61), 6000/(5.23 * 3.61 * 3.61));
        armRight = new MotorEx(hardwareMap, "arm-right",28*(5.23 * 3.61 * 3.61), 6000/(5.23 * 3.61 * 3.61));
        fold = new MotorEx(hardwareMap, "arm_fold",28*(5.23 * 3.61), 6000/(5.23 * 3.61));

        armLeft.setInverted(true);
        fold.setInverted(true);

        armLeft.resetEncoder();
        armRight.resetEncoder();
        fold.resetEncoder();

        armLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fold.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        armController = new VEPOSController(armLeft, armRight);
        foldController = new VEPOSController(fold);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        telemetry.addData("aLPos ", armLeft.getCurrentPosition());
        telemetry.addData("aRPos ", armRight.getCurrentPosition());
        telemetry.addData("fPos ", fold.getCurrentPosition());
        telemetry.addData("aLVelo", armLeft.getVelocity());
        telemetry.addData("aRVelo", armRight.getVelocity());
        telemetry.addData("fVelo", fold.getVelocity());
        telemetry.addData("aLPow", armLeft.motor.getPower());
        telemetry.addData("aRPow", armRight.motor.getPower());
        telemetry.addData("fPow", fold.motor.getPower());
        telemetry.addData("targetA", armPos);
        telemetry.addData("targetF", foldPos);
        telemetry.addData("totalErrA ", 0);
        telemetry.addData("totalErrF ", 0);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            long time = System.currentTimeMillis();

            foldController.setParameters(aKp, aKi, aKd, aKf, aKs, aKv, aKa, maxVelo, minVelo, maxTErr, aKvp, aKvi);

            armController.setTargetPosition(armPos);
            foldController.setTargetPosition(foldPos);
            armController.loop();
            foldController.loop();

            telemetry.addData("aLPos ", armLeft.getCurrentPosition());
            telemetry.addData("aRPos ", armRight.getCurrentPosition());
            telemetry.addData("fPos ", fold.getCurrentPosition());
            telemetry.addData("aLVelo", armLeft.getVelocity());
            telemetry.addData("aRVelo", armRight.getVelocity());
            telemetry.addData("fVelo", fold.getVelocity());
            telemetry.addData("aLPow", armLeft.motor.getPower());
            telemetry.addData("aRPow", armRight.motor.getPower());
            telemetry.addData("fPow", fold.motor.getPower());
            telemetry.addData("targetA", armPos);
            telemetry.addData("targetF", foldPos);
            telemetry.addData("totalErrA ", armController.totalErr);
            telemetry.addData("totalErrF ", foldController.totalErr);

            telemetry.update();

            sleep(100-(System.currentTimeMillis()-time));
        }
    }

}