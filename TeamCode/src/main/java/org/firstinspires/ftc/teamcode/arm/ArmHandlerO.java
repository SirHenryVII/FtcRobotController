//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//public class ArmHandlerO {
//    public static interface RunAfter {
//        void run();
//    }
//    public static enum State {
//        START,
//        GROUND,
//        LOW,
//        MIDDLE,
//        HIGH
//    }
//
//    private PIDController controllerA;
//    private PIDController controllerF;
//    private final Telemetry telemetry;
//
//    private DcMotorEx arm;
//    private DcMotorEx armFold;
//
//    private final double ticksInDegree = (28 * 18.9) / 360;
//    public double pA = 0.01, iA = 0, dA = 0.0004, fA = 0;
//    public double pF = 0.01, iF = 0, dF = 0.0004, fF = 0;
//
//
//
//
//    private State armState = State.START;
//    private boolean foldTrajectoryGreat = false;
//    private boolean armTrajectoryGreat = false;
//    private int targetA = 0;
//    private int targetF = 0;
//    private RunAfter runAfter = null;
//
//    public void setState(State state) {
//        final int beforeA = targetA;
//        final int beforeF = targetF;
//        switch (armState) {
//            case START: {
//                switch (state) {
//                    case GROUND: {
//                        pF = 0.006;
//                        targetF = 110;
//                        break;
//                    }
//                    case LOW: {
//                        pF = 0.005;
//                        pA = 0.01;
//                        targetA = 350;
//                        targetF = 205;
//                        break;
//                    }
//                }
//                break;
//            }
//            case GROUND:{
//                switch (state) {
//                    case START: {
//                        pF = 0.0075;
//                        targetF = 0;
//                        break;
//                    }
//                }
//                break;
//            }
//            case LOW: {
//                switch (state) {
//                    case START: {
//                        pF = 0.005;
//                        pA = 0.0025;
//                        targetA = 0;
//                        targetF = 0;
//                        break;
//                    }
//                    case GROUND: {
//                        pF = 0.005;
//                        pA = 0.0025;
//                        targetA = 0;
//                        targetF = 110;
//                        break;
//                    }
//                }
//            }
//        }
//        if(beforeA == targetA && beforeF == targetF) return;
//        foldTrajectoryGreat = armFold.getCurrentPosition() < targetF;
//        armTrajectoryGreat = arm.getCurrentPosition() < targetA;
//        armState = state;
//    }
//    public int getTargetArm() { return targetA; }
//    public void setTargetArm(int position) {
//        if(position > arm.getCurrentPosition()) pA = 0.01;
//        else pA = 0.0025;
//        targetA = position;
//    }
//
//
//
//    public int getTargetFold() { return targetF; }
//    public void setTargetFold(int position, double p) {
//        pF = p;
//        targetF = position;
//    }
//    public void setTargetFold(int position) {
//        pF = 0.01;
//        targetF = position;
//    }
//
//
//
//    public ArmHandlerO(Telemetry telemetry, DcMotorEx arm, DcMotorEx armFold){
//        this.telemetry = telemetry;
//        this.arm = arm;
//        this.armFold = armFold;
//    }
//
//    public void Init(){
//
//        //Innit Motors
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armFold.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setDirection(DcMotorEx.Direction.REVERSE);
//        armFold.setDirection(DcMotorEx.Direction.FORWARD);
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armFold.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//        //innit PID
//        controllerA = new PIDController(pA, iA, dA);
//        controllerF = new PIDController(pF, iF, dF);
//    }
//
//    public void Loop(){
//
//        //PID Loop
//        controllerA.setPID(pA, iA, dA);
//        controllerF.setPID(pF, iF, dF);
//
//        int armPos = arm.getCurrentPosition();
//        int armFoldPos = armFold.getCurrentPosition();
//
//        double pidA = controllerA.calculate(armPos, targetA);
//        double pidF = controllerF.calculate(armFoldPos, targetF);
//
//        double ffA = Math.cos(Math.toRadians(targetA / ticksInDegree)) * fA;
//        double ffF = Math.cos(Math.toRadians(targetF / ticksInDegree)) * fF;
//
//        if(pA != 0.01) {
//            boolean armState = arm.getCurrentPosition() > targetA;
//            if (armState == armTrajectoryGreat) pA = 0.01;
//        }
//
//        if(pF != 0.01) {
//            boolean foldState = armFold.getCurrentPosition() > targetF;
//            if (foldState == foldTrajectoryGreat) pF = 0.01;
//        }
//
//        arm.setPower(pidA + ffA);
//        armFold.setPower(pidF + ffF);
//
//        telemetry.addData("state", armState.name());
//        telemetry.addData("targetA", targetA);
//        telemetry.addData("posA", arm.getCurrentPosition());
//        telemetry.addData("pA", pA);
//        telemetry.addData("targetF", targetF);
//        telemetry.addData("posF", armFold.getCurrentPosition());
//        telemetry.addData("pF", pF);
//    }
//}
