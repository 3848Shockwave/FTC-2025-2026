package org.firstinspires.ftc.teamcode.Subsystems.Helpers;

public class TargetInfo {
    // This class stores information about a detected limelight target
    private double id;
    private double x;
    private double lastX;
    private double y;
    private double area;
    private double distance;
     // TO BE CALCULATED https://www.youtube.com/watch?v=Ap1lBywv00M brogan pratt video needed

    public TargetInfo(double id,double yaw, double pitch, double area) {
        this.x = yaw;
        this.y = pitch;
        this.area = area;
        this.id = id;
    }
    public double getTargetX(){
        return x;
    }
    public double getTargetLastX(){
        return lastX;
    }
    public double getTargetY(){
        return y;
    }
    public double getArea() {
        return area;
    }

    public double getDistance(){
        distance= 17.20075 *Math.pow(area,-0.509886);
        return distance;
    }
    public double getID() {
        return id;
    }

    public void setX(double newX){
        lastX = x;
        x = newX;
    }
    public void setY(double y){
        this.y = y;
    }
    public void setA(double area){
        this.area = area;
    }
   
}
