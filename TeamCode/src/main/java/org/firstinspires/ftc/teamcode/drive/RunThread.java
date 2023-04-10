package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.IController;

public class RunThread implements Runnable {
    final IController[] controllers;
    final LinearOpMode opMode;

    public RunThread(LinearOpMode opMode, IController... controllers) {
        this.controllers = controllers;
        this.opMode = opMode;
    }

    @Override
    public void run() {
        while(opMode.opModeIsActive()) {
            long time = System.currentTimeMillis();
            for(IController iController : controllers) iController.loop();

            try {
                sleep(100-(System.currentTimeMillis()-time));
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
