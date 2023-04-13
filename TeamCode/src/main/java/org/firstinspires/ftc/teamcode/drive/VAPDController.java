package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.fasterxml.jackson.core.json.DupDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.IController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class VAPDController {
    public ScalerMap scalerMap = new ScalerMap();


    private List<MotorEx> motors = new ArrayList<>();

    private final MotorEx topLeft;
    private final MotorEx topRight;
    private final MotorEx bottomLeft;
    private final MotorEx bottomRight;

    private Integer topLeftP = 1;
    private Integer topRightP = 1;
    private Integer bottomLeftP = 1;
    private Integer bottomRightP = 1;
    double power = 0.75;

    public VAPDController(MotorEx topLeft, MotorEx topRight, MotorEx bottomLeft, MotorEx bottomRight) {
        this.topLeft = topLeft; this.topRight = topRight; this.bottomLeft = bottomLeft; this.bottomRight = bottomRight;
        motors.add(topLeft); motors.add(topRight); motors.add(bottomLeft); motors.add(bottomRight);

        topLeft.setInverted(true);
        bottomRight.setInverted(true);

        motors.forEach(motorEx -> {
            motorEx.motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorEx.motorEx.setTargetPosition(1);
            motorEx.motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorEx.motorEx.setPower(power);
        });
    }

    public void strafe(int pos) {
        topLeftP -= pos;
        topRightP += pos;
        bottomLeftP += pos;
        bottomRightP -= pos;
        topLeft.motorEx.setTargetPosition(topLeftP); topRight.motorEx.setTargetPosition(topRightP);
        bottomRight.motorEx.setTargetPosition(bottomRightP); bottomLeft.motorEx.setTargetPosition(bottomLeftP);
    }
    public void spin(int pos) {
        topLeftP += pos;
        topRightP -= pos;
        bottomLeftP += pos;
        bottomRightP -= pos;
        topLeft.motorEx.setTargetPosition(topLeftP); topRight.motorEx.setTargetPosition(topRightP);
        bottomRight.motorEx.setTargetPosition(bottomRightP); bottomLeft.motorEx.setTargetPosition(bottomLeftP);
    }
    public void straight(int pos) {
        topLeftP += pos;
        topRightP += pos;
        bottomLeftP += pos;
        bottomRightP += pos;
        topLeft.motorEx.setTargetPosition(topLeftP); topRight.motorEx.setTargetPosition(topRightP);
        bottomRight.motorEx.setTargetPosition(bottomRightP); bottomLeft.motorEx.setTargetPosition(bottomLeftP);
    }

    public boolean isBusy() {
        return topLeft.motorEx.isBusy() || topRight.motorEx.isBusy()
                || bottomLeft.motorEx.isBusy() || bottomRight.motorEx.isBusy();
    }

}
