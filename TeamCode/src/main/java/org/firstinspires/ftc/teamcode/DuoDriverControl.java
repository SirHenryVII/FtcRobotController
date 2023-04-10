package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.ArmHandler;
import org.firstinspires.ftc.teamcode.drive.ArmThread;
import org.firstinspires.ftc.teamcode.drive.RunThread;

import java.util.concurrent.atomic.AtomicBoolean;


@TeleOp(name="DuoDriver Control")

public class    DuoDriverControl extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo rightClaw;
    private Servo leftClaw;
    private DcMotorEx top_left_drive;
    private DcMotorEx top_right_drive;
    private DcMotorEx bottom_left_drive;
    private DcMotorEx bottom_right_drive;
    private boolean reverseMode;
    private boolean slowMode;

    private ArmHandler armHandler;
    private AtomicBoolean armTickEnabled = new AtomicBoolean(true);

    @Override
    public void init() {
        rightClaw = hardwareMap.get(Servo.class, "right-claw");
        leftClaw = hardwareMap.get(Servo.class, "left-claw");
        top_left_drive = hardwareMap.get(DcMotorEx.class, "top_left_drive");
        top_right_drive = hardwareMap.get(DcMotorEx.class, "top_right_drive");
        bottom_left_drive = hardwareMap.get(DcMotorEx.class, "bottom_left_drive");
        bottom_right_drive = hardwareMap.get(DcMotorEx.class, "bottom_right_drive");

        armHandler = new ArmHandler(hardwareMap);

        //PID Setup

        //Motor Setups

        top_left_drive.setDirection(DcMotor.Direction.FORWARD);
        top_right_drive.setDirection(DcMotor.Direction.REVERSE);
        bottom_left_drive.setDirection(DcMotor.Direction.REVERSE);
        bottom_right_drive.setDirection(DcMotor.Direction.FORWARD);

        //bool inits
        reverseMode = false;
        slowMode = false;

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
        new Thread(new ArmThread(armTickEnabled, armHandler)).start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //Gamepad 1

        double div = 1.25;
        double finilize = 1;
        if(slowMode) div = 2.5;
        if(reverseMode) finilize = -1;

        //Drive Vars
        double forward = gamepad1.left_stick_y/div;
        double sides = gamepad1.left_stick_x/div;
        double spin = gamepad1.right_stick_x/div;

        //Drive functions
        top_left_drive.setPower((forward - sides + spin)*finilize);
        top_right_drive.setPower((forward + sides - spin)*finilize);
        bottom_left_drive.setPower((forward + sides + spin)*finilize);
        bottom_right_drive.setPower((forward - sides - spin)*finilize);

        //slow mode
        if(gamepad1.left_stick_button) slowMode = !slowMode;

        //reverse mode
        if(gamepad1.right_stick_button) reverseMode = !reverseMode;


        //Gamepad 2

        //Move to Start
        if(gamepad2.start){
            armHandler.setState(ArmHandler.State.START);
        }

        //Open and close claw
        if(gamepad2.left_bumper) {
            clawChange(true);
        }
        else if(gamepad2.right_bumper) {
            clawChange(false);
        }

        //Arm Fold Manual Control
        if(gamepad2.left_stick_y != 0) {
//            armHandlerO.setTargetFold((int) (armHandlerO.getTargetFold() - (gamepad2.left_stick_y)));
        }
        //Arm Manual Control
        if(gamepad2.right_stick_y != 0) {
//            armHandlerO.setTargetArm((int) (armHandlerO.getTargetArm() + (gamepad2.right_stick_y*2)));
        }

        //Move arm to ground level
        if(gamepad2.a){
            armHandler.setState(ArmHandler.State.GROUND);
        }

        //Move arm to lowest pole
        else if(gamepad2.b){
            armHandler.setState(ArmHandler.State.LOW);
        }

        //Move arm to medium pole
        else if(gamepad2.x){
            armHandler.setState(ArmHandler.State.MEDIUM);
        }

        //Move arm to highest pole
        else if(gamepad2.y){
            armHandler.setState(ArmHandler.State.HIGH);
        }

        //Both Gamepads
        if(gamepad1.back || gamepad2.back) {
//            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            Arm.setTargetPosition(0);
//            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm_fold.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            arm_fold.setTargetPosition(0);
//            arm_fold.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //Telemetry Output
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
        armTickEnabled.set(false);
    }

}
