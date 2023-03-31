package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmHandler {

    //Innit Global Variables
    private final Telemetry telemetry;
    private DcMotorEx arm;
    private DcMotorEx armFold;

    public double pA = 0.01, iA = 0, dA = 0.0004, fA = 0.2;
    public double pF = 0.01, iF = 0, dF = 0.0004, fF = 0;

    private final double ticksInDegree = (28 * 18.9) / 360;

    private int targetA = 0;

    public int getTargetArm() { return targetA; }
    public void setTargetArm(int position) {
        if(position > arm.getCurrentPosition()) pA = 0.01;
        else pA = 0.0025;
        targetA = position;
    }

    private int targetF = 0;
    public int getTargetFold() { return targetF; }
    public void setTargetFold(int position, double p) {
        pF = p;
        targetF = position;
    }
    public void setTargetFold(int position) {
        pF = 0.01;
        targetF = position;
    }

    private PIDController controllerA;
    private PIDController controllerF;

    public ArmHandler(Telemetry telemetry, DcMotorEx arm, DcMotorEx armFold){
        this.telemetry = telemetry;
        this.arm = arm;
        this.armFold = armFold;
    }

    public void Init(){

        //Innit Motors
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armFold.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorEx.Direction.REVERSE);
        armFold.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armFold.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //innit PID
        controllerA = new PIDController(pA, iA, dA);
        controllerF = new PIDController(pF, iF, dF);
    }

    public void Loop(){

        //PID Loop
        controllerA.setPID(pA, iA, dA);
        controllerF.setPID(pF, iF, dF);

        int armPos = arm.getCurrentPosition();
        int armFoldPos = armFold.getCurrentPosition();

        double pidA = controllerA.calculate(armPos, targetA);
        double pidF = controllerF.calculate(armFoldPos, targetF);

        double ffA = Math.cos(Math.toRadians(targetA / ticksInDegree)) * fA;
        double ffF = Math.cos(Math.toRadians(targetF / ticksInDegree)) * fF;

        if((pA == 0.0025 && arm.getCurrentPosition() < targetA) || arm.getCurrentPosition() <= 10){
            pA = 0.01;
        }

        arm.setPower(pidA + ffA);
        armFold.setPower(pidF + ffF);

        telemetry.addData("targetA", targetA);
        telemetry.addData("posA", arm.getCurrentPosition());
        telemetry.addData("pA", pA);
//        telemetry.addData("posA ", armPos);
//        telemetry.addData("posF ", armFoldPos);
//        telemetry.addData("targetA ", targetA);
//        telemetry.addData("targetF ", targetF);
//        telemetry.update();

    }
}
