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

public class DuoDriverControl extends OpMode
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
    private boolean reverseTrigger;
    private boolean slowMode;
    private boolean slowTrigger;
    private boolean fastFoldTrigger;
    private boolean fastFold = false;

    private long elapsed = System.currentTimeMillis();

    private ArmHandler armHandler;
//    private AtomicBoolean armTickEnabled = new AtomicBoolean(true);

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
//        new Thread(new ArmThread(armTickEnabled, armHandler)).start();
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
        double spin = -(gamepad1.right_stick_x/div);
        if(reverseMode) spin = spin*-1;

        //Drive functions
        top_left_drive.setPower((forward - sides + spin)*finilize);
        top_right_drive.setPower((forward + sides - spin)*finilize);
        bottom_left_drive.setPower((forward + sides + spin)*finilize);
        bottom_right_drive.setPower((forward - sides - spin)*finilize);

        //slow mode
        if(gamepad1.left_bumper && !slowTrigger) {
            slowMode = !slowMode;
            slowTrigger = true;
        }
        if(!gamepad1.left_bumper) slowTrigger = false;
        if(gamepad2.left_bumper && !fastFoldTrigger) {
            fastFoldTrigger = true;
            fastFold = !fastFold;
        }
        if(!gamepad2.left_bumper) fastFoldTrigger = false;

        //reverse mode
        if(gamepad1.right_bumper && !reverseTrigger) {
            reverseMode = !reverseMode;
            reverseTrigger = true;
        }
        if(!gamepad1.right_bumper) reverseTrigger = false;


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
        if(gamepad2.right_stick_y != 0) {
            armHandler.setPosition((int) (armHandler.armController.targetPosition+(gamepad2.right_stick_y*2)), armHandler.foldController.targetPosition);
        }
        //Arm Manual Control
        if(gamepad2.left_stick_y != 0) {
            double foldMult = 1.5;
            if(fastFold) foldMult = 3;
            armHandler.setPosition(armHandler.armController.targetPosition, (int) (armHandler.foldController.targetPosition+(gamepad2.left_stick_y*foldMult)));
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
            armHandler.armController.parent.resetEncoder();
            armHandler.armController.followers[0].resetEncoder();
            armHandler.foldController.parent.resetEncoder();
        }

        //Telemetry Output
        if(System.currentTimeMillis()-elapsed>=100) {
            armHandler.loop();
            elapsed = System.currentTimeMillis();
        }
        telemetry.addData("Status", "Run Time: " + runtime.milliseconds());
        telemetry.addData("FoldT", armHandler.foldController.targetPosition);
        telemetry.addData("Fold", armHandler.foldController.parent.getCurrentPosition());
        telemetry.update();
    }

    // Utility Functions
    private void clawChange(boolean open) {
        if (open) {
            rightClaw.setPosition(1);
            leftClaw.setPosition(0.5);
            return;
        }
        leftClaw.setPosition(1);
        rightClaw.setPosition(0.3);
    }
    @Override
    public void stop() {
//        armTickEnabled.set(false);
    }

}
