package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.IController;

import java.util.concurrent.atomic.AtomicBoolean;

public class RunThread implements Runnable {
    final IController[] controllers;
    final AtomicBoolean enable;

    public RunThread(AtomicBoolean enable, IController... controllers) {
        this.controllers = controllers;
        this.enable = enable;
    }

    @Override
    public void run() {
        while(enable.get()) {
            long time = System.currentTimeMillis();
            for(IController iController : controllers) iController.loop();

            try {
                sleep(150-(System.currentTimeMillis()-time));
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
