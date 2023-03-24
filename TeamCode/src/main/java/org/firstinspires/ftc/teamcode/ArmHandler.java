package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmHandler {

    //Innit Global Variables
    private DcMotorEx Arm;
    private DcMotorEx Arm_Fold;

    public double pA = 0, iA = 0, dA = 0, fA = 0;
    public double pF = 0, iF = 0, dF = 0, fF = 0;

    private final double ticks_in_degree = (28 * 18.9) / 360;

    public int targetA = 0;
    public int targetF = 0;

    private PIDController controllerA;
    private PIDController controllerF;

    public ArmHandler(DcMotorEx arm, DcMotorEx arm_fold){
        this.Arm = arm;
        this.Arm_Fold = arm_fold;
    }

    public void setArmTarget(int target){
        this.targetA = target;
    }

    public void setArmFoldTarget(int target){
        this.targetF = target;
    }

    public void Init(){

        //Innit Motors
        Arm.setDirection(DcMotorEx.Direction.REVERSE);
        Arm_Fold.setDirection(DcMotorEx.Direction.FORWARD);

        //innit PID
        controllerA = new PIDController(pA, iA, dA);
        controllerF = new PIDController(pF, iF, dF);
    }

    public void Loop(){

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
