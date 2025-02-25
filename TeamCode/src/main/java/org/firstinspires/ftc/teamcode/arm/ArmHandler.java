package org.firstinspires.ftc.teamcode.arm;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmHandler {
    public static enum State {
        HIGH(1100,1500),
        MEDIUM(900 ,1500),
        LOW(400,800),
        GROUND(0,500),
        START(0,0);
        public final int arm;
        public final int fold;

        State(int arm, int fold) {
            this.arm = arm;
            this.fold = fold;
        }
    }

    public final VEPOSController armController;
    public final VEPOSController foldController;

    public ArmHandler(HardwareMap hardwareMap) {
        MotorEx armLeft = new MotorEx(hardwareMap, "arm-left",28*(5.23 * 3.61 * 3.61), 6000/(5.23 * 3.61 * 3.61));
        MotorEx armRight = new MotorEx(hardwareMap, "arm-right",28*(5.23 * 3.61 * 3.61), 6000/(5.23 * 3.61 * 3.61));
        MotorEx fold = new MotorEx(hardwareMap, "arm_fold",28*(5.23 * 3.61), 6000/(5.23 * 3.61));

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

        //Arm values are default
        foldController.setParameters(0.00332, 0.00254, 0.00322, 0,0,1,0,2500,-2500,1500,5,0.4);
    }

    public void setState(State armState) {
        armController.setTargetPosition(armState.arm);
        foldController.setTargetPosition(armState.fold);
    }

    public void setPosition(int arm, int fold) {
        armController.setTargetPosition(arm);
        foldController.setTargetPosition(fold);
    }

    public void loop() {
        armController.loop();
        foldController.loop();
    }

    public boolean isBusy() {
        return armController.parent.motorEx.isBusy() || foldController.parent.motorEx.isBusy();
    }
}
