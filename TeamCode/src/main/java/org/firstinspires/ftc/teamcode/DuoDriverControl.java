package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="DuoDriver Control")

public class DuoDriverControl extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Arm;
    private Servo rightClaw;
    private Servo leftClaw;
    private DcMotor arm_fold;
    private DcMotor top_left_drive;
    private DcMotor top_right_drive;
    private DcMotor bottom_left_drive;
    private DcMotor bottom_right_drive;
    private int arm_target;
    private int arm_fold_target;
    private PIDFController pidf;


    @Override
    public void init() {
        rightClaw = hardwareMap.get(Servo.class, "right-claw");
        leftClaw = hardwareMap.get(Servo.class, "left-claw");
        Arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm_fold = hardwareMap.get(DcMotorEx.class, "arm_fold");
        top_left_drive = hardwareMap.get(DcMotor.class, "top_left_drive");
        top_right_drive = hardwareMap.get(DcMotor.class, "top_right_drive");
        bottom_left_drive = hardwareMap.get(DcMotor.class, "bottom_left_drive");
        bottom_right_drive = hardwareMap.get(DcMotor.class, "bottom_right_drive");

        //PID Setup
        arm_target = 0;
        arm_fold_target = 0;


        //Motor Setups
        Arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setDirection(DcMotorEx.Direction.FORWARD);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm_fold.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_fold.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_fold.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_fold.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        top_left_drive.setDirection(DcMotor.Direction.REVERSE);
        top_right_drive.setDirection(DcMotor.Direction.FORWARD);
        bottom_left_drive.setDirection(DcMotor.Direction.FORWARD);
        bottom_right_drive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        //Gamepad 1

        //Drive Vars
        double forward = gamepad1.left_stick_y/1.25;
        double sides = gamepad1.left_stick_x/1.25;
        double spin = gamepad1.right_stick_x/1.75;

        //Drive functionality
        top_left_drive.setPower(forward + sides + spin);
        top_right_drive.setPower(forward - sides - spin);
        bottom_left_drive.setPower(forward - sides + spin);
        bottom_right_drive.setPower(forward + sides - spin);

        //Gamepad 2

        //Move to Start
        if(gamepad2.start){
            arm_move(0);
            arm_fold_move(0);
        }

        //Open and close claw
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
        //Arm Manual Control
        if(gamepad2.right_stick_y != 0) {
            arm_move((int) (Arm.getCurrentPosition() + (gamepad2.right_stick_y*35)));
        }

        //Move arm to ground level
        if(gamepad2.a){
            arm_move(0);
            arm_fold_move(152);
        }

        //Move arm to lowest pole
        else if(gamepad2.b){
            arm_move(531);
            arm_fold_move(264);
        }

        //Move arm to medium pole
        else if(gamepad2.x){
            arm_move(694);
            arm_fold_move(280);
        }

        //Move arm to highest pole
        else if(gamepad2.y){
            arm_move(849);
            arm_fold_move(281);
        }

        //Both Gamepads
        if(gamepad1.back || gamepad2.back) {
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setTargetPosition(0);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_fold.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm_fold.setTargetPosition(0);
            arm_fold.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //PID Logic

        //Telemetry Output
        telemetry.addData("Arm_fold", arm_fold.getCurrentPosition());
        telemetry.addData("Arm", Arm.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime.milliseconds());
        telemetry.update();
    }

    // Utility Functions
    private void arm_move(int target)
    {
        Arm.setTargetPosition(target);
        Arm.setPower(0.4);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void arm_fold_move(int target)
    {
        arm_fold.setTargetPosition(target);
        arm_fold.setPower(0.4);
        arm_fold.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void clawChange(boolean bool) {
        if (bool) {
            rightClaw.setPosition(0.7);
            leftClaw.setPosition(0.5);
            return;
        }
        leftClaw.setPosition(1);
        rightClaw.setPosition(0);
    }
    @Override
    public void stop() {
    }

}
