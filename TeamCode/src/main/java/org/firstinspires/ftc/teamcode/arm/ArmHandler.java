//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.controller.PController;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@Config
//@TeleOp(name = "Arm Test")
//public class ArmHandler {
//    private final Motor arm;
//    private final Motor fold;
//
//    public double pA = 0.01, iA = 0, dA = 0.0004, fA = 0;
//    public double pF = 0.01, iF = 0, dF = 0.0004, fF = 0;
//
//    final PIDFController armController;
//    final PIDFController foldController;
//
//    public int armTarget;
//    public int foldTarget;
//
//    public ArmHandler(Motor arm, Motor fold) {
//        arm.setRunMode(Motor.RunMode.VelocityControl);
//        fold.setRunMode(Motor.RunMode.VelocityControl);
//
//        this.arm = arm;
//        this.fold = fold;
//
//    }
//
//
//    public void loop() {
//        armController.setP(pA); armController.setI(iA); armController.setD(dA); armController.setF(fA);
//        foldController.setP(fA); armController.setI(fA); armController.setD(fA); armController.setF(fA);
//
//        double armOut = armController.calculate(arm.getCurrentPosition(), armTarget);
//        double foldOut = armController.calculate(fold.getCurrentPosition(), foldTarget);
//
//        PController armFController = new PController(pA);
//        PController foldFController = new PController(pF);
//
//        armFController.setSetPoint(armTarget);
//        foldFController.setSetPoint(foldTarget);
//
//        while (!armFController.atSetPoint()) {
//            double output = armFController.calculate(
//                    arm.getCurrentPosition()
//            );
//            arm(output);
//        }
//        m_motor.stopMotor();
//    }
//}
