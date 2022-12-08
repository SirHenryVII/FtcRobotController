package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name="Driver Control")

public class DriverControl extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor arm;
    private Servo rightClaw;
    private Servo leftClaw;
    private DcMotor arm_fold;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        rightClaw = hardwareMap.get(Servo.class, "left-claw");
        leftClaw = hardwareMap.get(Servo.class, "right-claw");
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm_fold = hardwareMap.get(DcMotor.class, "arm_fold");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Delivery", "AndroidStudio");

        //Motor Setups
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_fold.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_fold.setTargetPosition(0);
        arm_fold.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        double x;
        double y;

        //Get Joystick Inputs
        x  = gamepad1.right_stick_x;
        y = -gamepad1.right_stick_y;
        // Send calculated power to wheels
        leftDrive.setPower(y + x);
        rightDrive.setPower(y - x);

        if(gamepad1.left_bumper) {
            clawChange(false);
        }
        else if(gamepad1.right_bumper) {
            clawChange(true);
        }

        //Arm Move Code
        if(gamepad1.a){
            arm_move(0);
            arm_fold_move(390);
        }
        else if(gamepad1.b){
            arm_move(175);
        }
        else if(gamepad1.y){
            arm_move(250);
        }
        else if(gamepad1.x){
            arm_move(750);
            arm_fold_move(655);
        }

        //Arm Fold Manual Control
        if(gamepad1.dpad_down) {arm_fold_move(arm_fold.getCurrentPosition() + 25);}
        else if(gamepad1.dpad_up) {arm_fold_move(arm_fold.getCurrentPosition() - 25);}

        //Arm manual control
        if(gamepad1.dpad_left) {arm_move(arm.getCurrentPosition() + 25);}
        else if(gamepad1.dpad_right) {arm_move(arm.getCurrentPosition() - 25);}

        //Telemetry Output
        telemetry.addData("Arm_fold", arm_fold.getCurrentPosition());
        telemetry.addData("lclawpos", leftClaw.getPosition());
        telemetry.addData("rclawpos", rightClaw.getPosition());
        telemetry.addData("Status", "Run Time: " + runtime.milliseconds());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", x, y);
        telemetry.update();
    }

    // Utility Functions
    private void arm_move(int target)
    {
        arm.setTargetPosition(target);
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void arm_fold_move(int target)
    {
        arm_fold.setTargetPosition(target);
        arm_fold.setPower(1);
        arm_fold.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void clawChange(boolean bool) {
        if (bool) {
            rightClaw.setPosition(-0.4);
            leftClaw.setPosition(0.4);
            return;
        }
    }
    @Override
    public void stop() {
    }

}
