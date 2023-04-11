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

public class VAPDController implements IController {
    public ScalerMap scalerMap = new ScalerMap();


    private List<DcMotor> motors = new ArrayList<>();
    private List<Integer> positions = new ArrayList<>();

    private final DcMotor topLeft;
    private final DcMotor topRight;
    private final DcMotor bottomLeft;
    private final DcMotor bottomRight;

    private Integer topLeftP = 1;
    private Integer topRightP = 1;
    private Integer bottomLeftP = 1;
    private Integer bottomRightP = 1;

    public VAPDController(DcMotor topLeft, DcMotor topRight, DcMotor bottomLeft, DcMotor bottomRight) {
        this.topLeft = topLeft; this.topRight = topRight; this.bottomLeft = bottomLeft; this.bottomRight = bottomRight;
        motors.add(topLeft); motors.add(topRight); motors.add(bottomLeft); motors.add(bottomRight);
        positions.add(topLeftP); positions.add(topRightP); positions.add(bottomLeftP); positions.add(bottomRightP);

        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.forEach(motorEx -> {
            motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        });
        topLeft.setTargetPosition(topLeftP); topRight.setTargetPosition(topRightP);
        bottomRight.setTargetPosition(bottomRightP); bottomLeft.setTargetPosition(bottomLeftP);
        motors.forEach(motorEx -> {
            motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorEx.setPower(0.5);
        });
    }

    public void strafe(int pos) {
        topLeftP -= pos;
        topRightP += pos;
        bottomLeftP += pos;
        bottomRightP -= pos;
        positions.clear();
        positions.add(topLeftP); positions.add(topRightP); positions.add(bottomLeftP); positions.add(bottomRightP);
    }
    public void spin(int pos) {
        topLeftP -= pos;
        topRightP += pos;
        bottomLeftP -= pos;
        bottomRightP += pos;
        positions.clear();
        positions.add(topLeftP); positions.add(topRightP); positions.add(bottomLeftP); positions.add(bottomRightP);
    }
    public void straight(int pos) {
        topLeftP += pos;
        topRightP += pos;
        bottomLeftP += pos;
        bottomRightP += pos;
        positions.clear();
        positions.add(topLeftP); positions.add(topRightP); positions.add(bottomLeftP); positions.add(bottomRightP);
    }

    public void loop() {
        topLeft.setTargetPosition(topLeftP); topRight.setTargetPosition(topRightP);
        bottomRight.setTargetPosition(bottomRightP); bottomLeft.setTargetPosition(bottomLeftP);
//        int i = 3;
//        for(int i = 0; i < 4; i++) {
//            MotorEx target = motors.get(i);
//            Integer posT = positions.get(i);
//
//            int cPos = target.getCurrentPosition();
//            int posError = posT - cPos;
//
//            double perc = 0;
//            if(posT != 0) perc = (float) cPos / (float) posT;
//            double scaler = scalerMap.get(Math.abs(perc));
//
//            System.out.println("Motor: " + i + " | " + posError + "(" + cPos + ") | " + scaler + " | % | " + perc );
//            target.setVelocity(posError * Math.abs(scaler));
//        }
    }

}
