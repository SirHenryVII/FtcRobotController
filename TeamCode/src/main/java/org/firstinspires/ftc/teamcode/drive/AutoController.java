package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IController;
import org.firstinspires.ftc.teamcode.arm.VEPOSController;

@Config
@TeleOp(name = "AutoController")
public class AutoController extends LinearOpMode {
    VAPDController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new VAPDController(
                new MotorEx(hardwareMap, "top_left_drive",28*(5.23 * 3.61), 6000/(5.23 * 3.61)),
                new MotorEx(hardwareMap, "top_right_drive",28*(5.23 * 3.61), 6000/(5.23 * 3.61)),
                new MotorEx(hardwareMap, "bottom_left_drive",28*(5.23 * 3.61), 6000/(5.23 * 3.61)),
                new MotorEx(hardwareMap, "bottom_right_drive",28*(5.23 * 3.61), 6000/(5.23 * 3.61)));
        controller.scalerMap.add(0, 0.1);
        controller.scalerMap.add(.25, 1);
        controller.scalerMap.add(.75, 1);
        controller.scalerMap.add(1.0, 0.1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        waitForStart();

//        new Thread(new RunThread(this, controller)).start();

        controller.straight(1000);

        while(opModeIsActive()) {
            long time = System.currentTimeMillis();
            controller.loop();

            sleep(150-(System.currentTimeMillis()-time));
        }
    }

}