package org.firstinspires.ftc.teamcode.opmode;
import org.firstinspires.ftc.teamcode.config.subsystems.testClaw;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
public class testTeleOp extends OpMode {
    public testClaw testClaw;

    @Override
    public void init(){
        testClaw = new testClaw(hardwareMap);
    }

    @Override
    public void loop(){
        if (gamepad1.x){
            testClaw.open();
        }

        if (gamepad1.y){
            testClaw.close();
        }
    }
}
