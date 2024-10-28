package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ProgrammingBoard {
    private DigitalChannel touchSensor;
    private DcMotor motor;
    private double ticksPerRotation;
    private Servo servo;

    public void init(HardwareMap hardwaremap) {
        touchSensor = hardwaremap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        motor = hardwaremap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE (stops motor) or FLOAT (power just turns off)
        motor.setDirection(DcMotorSimple.Direction.REVERSE); //FORWARD or REVERSE
        ticksPerRotation = motor.getMotorType().getTicksPerRev();

        servo = hardwaremap.get(Servo.class, "servo");
    }

    public boolean isTouchSensorPressed() {
        return !touchSensor.getState();
    }

    public boolean isTouchSensorReleased() {
        return touchSensor.getState();
    }

    public void setMotorSpeed(double speed) { //-1.0 is full speed backwards, 1.0 is full speed forwards, and 0.0 means zero power
        motor.setPower(speed);
    }

    public double getMotorRotations() {
        return motor.getCurrentPosition() / ticksPerRotation; //total number of ticks divided by number of ticks in a rotation
    }

    public void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setServoPosition(double position) { //position is a double between 0.0 & 1.0; you can also change the range or flip the direction of the range, but it's not needed
        servo.setPosition(position);
    }
}
