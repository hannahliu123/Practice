package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.ProgrammingBoard;

@TeleOp
public class ToggleButton extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();

    boolean prevButtonPressed;
    boolean motorOn;

    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.a && !prevButtonPressed) { //if button a is pressed and prevButtonPressed is false
            motorOn = !motorOn;
            telemetry.addData("Motor On", motorOn);
            if(motorOn) {
                board.setMotorSpeed(0.5);
            } else{
                board.setMotorSpeed(0.0);
            }
        }
        prevButtonPressed = gamepad1.a;

        //The first time you press the button, the motor turns on. The next time you press it, the motor turns off.
    }
}
