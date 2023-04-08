package org.firstinspires.ftc.teamcode.arm;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;

public class VEPOSController {
    final MotorEx parent;
    final MotorEx[] followers;

    public int targetPosition;
    public double aKp = 0.00332, aKi = 0.00254, aKd = 0.00322, aKf = 0, aKs = 0, aKv = 1, aKa = 0;
    public double maxVelo = 2500, minVelo = -1000, maxTErr = 1500, aKvp = 3.5, aKvi = 0.4;
    public int totalErr;


    public VEPOSController(MotorEx parent, MotorEx... followers) {
        this.parent = parent;
        this.followers = followers;

        parent.setRunMode(Motor.RunMode.VelocityControl);
        Arrays.stream(followers).forEach(motorEx -> motorEx.setRunMode(Motor.RunMode.RawPower));
    }

    public void setTargetPosition(int position) {
        this.targetPosition = position;
    }

    public void loop() {
        parent.setFeedforwardCoefficients(aKs, aKv, aKa);
        parent.setVeloCoefficients(aKp, aKi, aKd);

        int cPos = parent.getCurrentPosition();
        int posError = targetPosition - cPos;
        totalErr += posError;

        if(totalErr > maxTErr) totalErr = (int) maxTErr;
        else if (totalErr < -maxTErr) totalErr = -(int) maxTErr;

        double targetVelocity = posError * aKvp + (totalErr * aKvi);

        if(targetVelocity > maxVelo) targetVelocity = maxVelo;
        else if(targetVelocity < minVelo) targetVelocity = minVelo;


        parent.setVelocity(targetVelocity);
        double follPow = parent.motorEx.getPower();
        Arrays.stream(followers).forEach(motorEx -> motorEx.set(follPow));
    }

    public void setParameters(double aKp, double aKi, double aKd, double aKf, double aKs, double aKv, double aKa, double maxVelo, double minVelo, double maxTErr, double aKvp, double aKvi) {
        this.aKp = aKp;
        this.aKi = aKi;
        this.aKd = aKd;
        this.aKf = aKf;
        this.aKs = aKs;
        this.aKv = aKv;
        this.aKa = aKa;
        this.maxVelo = maxVelo;
        this.minVelo = minVelo;
        this.maxTErr = maxTErr;
        this.aKvp = aKvp;
        this.aKvi = aKvi;
    }
}
