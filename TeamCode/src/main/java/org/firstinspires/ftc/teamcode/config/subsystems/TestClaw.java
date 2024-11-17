package org.firstinspires.ftc.teamcode.config.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.util.action.RunAction;

public class TestClaw {

    public Servo servo;
    public RunAction open, close;

    public TestClaw(HardwareMap hi){
        servo = hi.get(Servo.class, "servo");

        open = new RunAction(this::open);
        close = new RunAction(this::close);
    }

    public void open(){
        servo.setPosition(0);
    }

    public void close(){
        servo.setPosition(100);
    }

}

