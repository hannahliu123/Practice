package org.firstinspires.ftc.teamcode.opmode;
import org.firstinspires.ftc.teamcode.config.subsystems.ProgrammingBoard;
import org.firstinspires.ftc.teamcode.config.subsystems.testClaw;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testTeleOp extends OpMode {
    public testClaw testClaw;
    ProgrammingBoard board = new ProgrammingBoard();
    String touchSensor;

    @Override
    public void init(){
        testClaw = new testClaw(hardwareMap);
        board.init(hardwareMap);
    }

    @Override
    public void loop(){
        if (gamepad1.x){
            testClaw.open();
        }
        if (gamepad1.y){
            testClaw.close();
        }

        if (board.isTouchSensorPressed()){
            touchSensor = "Pressed";
            board.setMotorSpeed(0.5);
        } else {
            touchSensor = "Not Pressed";
            board.setMotorSpeed(0.0);
        }
        telemetry.addData("Touch Pressed", touchSensor);

        double motorSpeed = gamepad1.left_stick_y;
        board.setMotorSpeed(motorSpeed);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Motor Rotations", board.getMotorRotations());

        if (gamepad1.a) {
            board.setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            board.setServoPosition(1.0);
        }
        else if (gamepad1.b) { //need the else if for the servo to prevent confusion
            board.setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            board.setServoPosition(0.0);
        }
        else {
            board.setServoPosition(0.5);
        }

        board.setServoPosition(gamepad1.left_trigger);
    }
}
