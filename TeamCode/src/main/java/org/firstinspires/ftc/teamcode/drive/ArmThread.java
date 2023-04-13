package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.IController;
import org.firstinspires.ftc.teamcode.arm.ArmHandler;

import java.util.concurrent.atomic.AtomicBoolean;

public class ArmThread implements Runnable {
    final ArmHandler armHandler;
    final AtomicBoolean enable;

    public ArmThread(AtomicBoolean enable, ArmHandler armHandler) {
        this.armHandler = armHandler;
        this.enable = enable;
    }

    @Override
    public void run() {
        while(enable.get()) {
            long time = System.currentTimeMillis();
            armHandler.loop();

            try {
                sleep(150-(System.currentTimeMillis()-time));
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
