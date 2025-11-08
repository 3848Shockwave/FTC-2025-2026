package org.firstinspires.ftc.teamcode.Subsystems.Helpers;

public class TargetInfo {
    public double id;
    public double x;
    public double y;
    public double area;

    public TargetInfo(double id,double yaw, double pitch, double area) {
        this.x = yaw;
        this.y = pitch;
        this.area = area;
        this.id = id;
    }
    public double getTargetX(){
        return x;
    }
    public double getTargetY(){
        return y;
    }
    public double getArea() {
        return area;
    }
    public double getID() {
        return id;
    }

    public void setX(double x){
        this.x = x;
    }
    public void setY(double y){
        this.y = y;
    }
    public void setA(double area){
        this.area = area;
    }
   
}
