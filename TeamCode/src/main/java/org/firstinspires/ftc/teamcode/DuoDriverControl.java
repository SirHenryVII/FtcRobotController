package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="DuoDriver Control")

public class DuoDriverControl extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor leftArm;
    private DcMotor rightArm;
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
        leftArm = hardwareMap.get(DcMotor.class, "left-arm");
        rightArm = hardwareMap.get(DcMotor.class, "right-arm");
        arm_fold = hardwareMap.get(DcMotor.class, "arm_fold");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Delivery", "AndroidStudio");

        //Motor Setups
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setTargetPosition(0);
        rightArm.setTargetPosition(0);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
//        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = gamepad1.right_stick_x;
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio, but only when
//        // at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//
//        motorFrontLeft.setPower(frontLeftPower);
//        motorBackLeft.setPower(backLeftPower);
//        motorFrontRight.setPower(frontRightPower);
//        motorBackRight.setPower(backRightPower);

        if(gamepad1.back && gamepad2.back) {
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftArm.setTargetPosition(0);
            rightArm.setTargetPosition(0);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_fold.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm_fold.setTargetPosition(0);
            arm_fold.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //Gamepad 1

        if(gamepad1.left_stick_y != 0 && gamepad1.right_stick_y != 0)

        //Gamepad 2
        if(gamepad2.start){
            arm_move(0);
            arm_fold_move(0);
        }

        if(gamepad2.left_bumper) {
            clawChange(false);
        }
        else if(gamepad2.right_bumper) {
            clawChange(true);
        }

        //Arm Fold Manual Control
        if(gamepad2.left_stick_y != 0) {
            arm_fold_move((int) (arm_fold.getCurrentPosition() + (gamepad2.left_stick_y*35)));
        }
        if(gamepad2.right_stick_y != 0) {
            arm_move((int) (leftArm.getCurrentPosition() + (gamepad2.right_stick_y*35)));
        }

        if(gamepad2.a){
            arm_move(0);
            arm_fold_move(388);
        }
        else if(gamepad2.b){
            arm_move(420);
            arm_fold_move(610);
        }
        else if(gamepad2.y){
            arm_move(550);
            arm_fold_move(590);
        }
        else if(gamepad2.x){
            arm_move(750);
            arm_fold_move(615);
        }

        //Telemetry Output
        telemetry.addData("Arm_fold", arm_fold.getCurrentPosition());
        telemetry.addData("lclawpos", leftClaw.getPosition());
        telemetry.addData("rclawpos", rightClaw.getPosition());
        telemetry.addData("Status", "Run Time: " + runtime.milliseconds());
        telemetry.update();
    }

    // Utility Functions
    private void arm_move(int target)
    {
        leftArm.setTargetPosition(target);
        rightArm.setTargetPosition(-target);
        leftArm.setPower(1);
        rightArm.setPower(1);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void arm_fold_move(int target)
    {
        arm_fold.setTargetPosition(target);
        arm_fold.setPower(1);
        arm_fold.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void clawChange(boolean bool) {
        if (bool) {
            leftClaw.setDirection(Servo.Direction.REVERSE);
            rightClaw.setDirection(Servo.Direction.REVERSE);
            rightClaw.setPosition(0.3);
            leftClaw.setPosition(0.7);
            return;
        }
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
    }
    @Override
    public void stop() {
    }

}
