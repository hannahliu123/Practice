package org.firstinspires.ftc.teamcode.config.subsystems;

public class Testing {
    double angle;
    double x;
    double y;

    public Testing(double angle){
        this.angle = angle;
    }

    public void setAngle(double angle){
        this.angle = angle;
    }

    public double getAngle(){
        return angle;
    }

    public double getX(){
        return x;
    }

    public void changeX(double change){
        x += change;
    }

    public setX(double x){
        this.x = x;
    }

    public void changeY(double change){
        y += change;
    }
}
