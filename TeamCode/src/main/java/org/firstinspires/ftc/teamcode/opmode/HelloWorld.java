package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Testing;

@TeleOp //This is an annotation that just tells the DS this is a TeleOp OpMode
public class HelloWorld extends OpMode {
    Testing robotLocation = new Testing(0);

    @Override
    public void init(){
        robotLocation.setAngle(0);
    }

    @Override
    public void loop(){
        if(gamepad1.dpad_left){
            robotLocation.changeX(-0.1);
        } else if(gamepad1.dpad_right){
            robotLocation.changeX(0.1);
        }
        if(gamepad1.dpad_up){
            robotLocation.changeY(0.1);
        } else if(gamepad1.dpad_down){
            robotLocation.changeY(-0.1);
        }

        telemetry.addData("Angle", robotLocation.getAngle());
    }
}
