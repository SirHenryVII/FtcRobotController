package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.fasterxml.jackson.core.json.DupDetector;

import org.firstinspires.ftc.teamcode.IController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class VAPDController implements IController {
    public ScalerMap scalerMap = new ScalerMap();


    private List<MotorEx> motors = new ArrayList<>();
    private List<Integer> positions = new ArrayList<>();

    private final MotorEx topLeft;
    private final MotorEx topRight;
    private final MotorEx bottomLeft;
    private final MotorEx bottomRight;

    private Integer topLeftP = 0;
    private Integer topRightP = 0;
    private Integer bottomLeftP = 0;
    private Integer bottomRightP = 0;

    public VAPDController(MotorEx topLeft, MotorEx topRight, MotorEx bottomLeft, MotorEx bottomRight) {
        this.topLeft = topLeft; this.topRight = topRight; this.bottomLeft = bottomLeft; this.bottomRight = bottomRight;
        motors.add(topLeft); motors.add(topRight); motors.add(bottomLeft); motors.add(bottomRight);
        positions.add(topLeftP); positions.add(topRightP); positions.add(bottomLeftP); positions.add(bottomRightP);

        topRight.setInverted(true);
        bottomLeft.setInverted(true);

        motors.forEach(motorEx -> {
            motorEx.resetEncoder();
            motorEx.setRunMode(Motor.RunMode.VelocityControl);
            motorEx.setFeedforwardCoefficients(0,1,0);
            motorEx.setVeloCoefficients(0.00332, 0.00254, 0.00322);
        });
    }

    public void strafe(int pos) {
        topLeftP -= pos;
        topRightP += pos;
        bottomLeftP += pos;
        bottomRightP -= pos;
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
    }

    public void loop() {
//        int i = 3;
        for(int i = 0; i < 4; i++) {
            MotorEx target = motors.get(i);
            Integer posT = positions.get(i);

            int cPos = target.getCurrentPosition();
            int posError = posT - cPos;

            double perc = 0;
            if(posT != 0) perc = (float) cPos / (float) posT;
            double scaler = scalerMap.get(Math.abs(perc));

            System.out.println("Motor: " + i + " | " + posError + "(" + cPos + ") | " + scaler + " | % | " + perc );
            target.setVelocity(posError * Math.abs(scaler));
//            target.setVelocity(posError);
        }
    }

}
