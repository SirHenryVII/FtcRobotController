package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.arm.ArmHandler;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
@Disabled
@TeleOp(name = "AutoController")
public class AutoController extends OpMode {
    ArmHandler armHandler;
    VAPDController driveController;
    Camera camera;
    private final AtomicBoolean threadingEnabled = new AtomicBoolean(true);

    @Override
    public void init() {
        driveController = new VAPDController(
                new MotorEx(hardwareMap, "top_left_drive",28*(5.23 * 3.61), 6000/(5.23 * 3.61)),
                new MotorEx(hardwareMap, "top_right_drive",28*(5.23 * 3.61), 6000/(5.23 * 3.61)),
                new MotorEx(hardwareMap, "bottom_left_drive",28*(5.23 * 3.61), 6000/(5.23 * 3.61)),
                new MotorEx(hardwareMap, "bottom_right_drive",28*(5.23 * 3.61), 6000/(5.23 * 3.61)));
        driveController.scalerMap.add(0, 0.1);
        driveController.scalerMap.add(.25, 1);
        driveController.scalerMap.add(.75, 1);
        driveController.scalerMap.add(1.0, 0.1);

        armHandler = new ArmHandler(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        camera = new Camera(this);



        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        new Thread(new ArmThread(threadingEnabled, armHandler)).start();
    }

    @Override
    public void loop() {
        try {
            driveController.straight(1000);
            sleep(5000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}