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

import org.checkerframework.checker.units.qual.A;


@TeleOp(name="DuoDriver Control")

public class DuoDriverControl extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx Arm;
    private Servo rightClaw;
    private Servo leftClaw;
    private DcMotorEx arm_fold;
    private DcMotorEx top_left_drive;
    private DcMotorEx top_right_drive;
    private DcMotorEx bottom_left_drive;
    private DcMotorEx bottom_right_drive;
private ArmHandler armHandler;

    @Override
    public void init() {
        rightClaw = hardwareMap.get(Servo.class, "right-claw");
        leftClaw = hardwareMap.get(Servo.class, "left-claw");
        Arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm_fold = hardwareMap.get(DcMotorEx.class, "arm_fold");
        top_left_drive = hardwareMap.get(DcMotorEx.class, "top_left_drive");
        top_right_drive = hardwareMap.get(DcMotorEx.class, "top_right_drive");
        bottom_left_drive = hardwareMap.get(DcMotorEx.class, "bottom_left_drive");
        bottom_right_drive = hardwareMap.get(DcMotorEx.class, "bottom_right_drive");

        //PID Setup
        armHandler = new ArmHandler(Arm, arm_fold);

        //Motor Setups

        armHandler.Init();

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
            armHandler.setArmTarget(0);
            armHandler.setArmFoldTarget(0);
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
            armHandler.setArmFoldTarget((int) (arm_fold.getCurrentPosition() + (gamepad2.left_stick_y*35)));
        }
        //Arm Manual Control
        if(gamepad2.right_stick_y != 0) {
            armHandler.setArmTarget((int) (Arm.getCurrentPosition() + (gamepad2.right_stick_y*35)));
        }

        //Move arm to ground level
        if(gamepad2.a){
            armHandler.setArmTarget(0);
            armHandler.setArmFoldTarget(152);
        }

        //Move arm to lowest pole
        else if(gamepad2.b){
            armHandler.setArmTarget(531);
            armHandler.setArmFoldTarget(264);
        }

        //Move arm to medium pole
        else if(gamepad2.x){
            armHandler.setArmTarget(694);
            armHandler.setArmFoldTarget(280);
        }

        //Move arm to highest pole
        else if(gamepad2.y){
            armHandler.setArmTarget(894);
            armHandler.setArmFoldTarget(281);
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

        //Telemetry Output
        telemetry.addData("Arm_fold", arm_fold.getCurrentPosition());
        telemetry.addData("Arm", Arm.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime.milliseconds());
        telemetry.update();
    }

    // Utility Functions
    private void clawChange(boolean open) {
        if (open) {
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
