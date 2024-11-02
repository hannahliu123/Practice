package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ProgrammingBoard {
    private DigitalChannel touchSensor;
    private DcMotor motor;
    private double ticksPerRotation;
    private Servo servo;
    private AnalogInput pot;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    public void init(HardwareMap hardwaremap) {
        touchSensor = hardwaremap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        motor = hardwaremap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE (stops motor) or FLOAT (power just turns off)
        motor.setDirection(DcMotorSimple.Direction.REVERSE); //FORWARD or REVERSE
        ticksPerRotation = motor.getMotorType().getTicksPerRev();

        servo = hardwaremap.get(Servo.class, "servo");

        pot = hardwaremap.get(AnalogInput.class, "potentiometer");

        colorSensor = hardwaremap.get(ColorSensor.class, "color_distance_sensor");
        distanceSensor = hardwaremap.get(DistanceSensor.class, "color_distance_sensor");
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

    public double getPotAngle() {
        return Range.scale(pot.getVoltage(), 0, pot.getMaxVoltage(), 0, 270); //LearnJavaForFTC Lesson 9.2
    }

    public double potRange() {
        return Range.scale(pot.getVoltage(), 0, pot.getMaxVoltage(), 0.0, 1.0); //LearnJavaForFTC Lesson 9.2
    }

    public int getAmountRed() {
        return colorSensor.red();
    }

    public double getDistance(DistanceUnit du) {
        return distanceSensor.getDistance(du);
    }

    public int getAmountBlue() {
        return colorSensor.blue();
    }
}
