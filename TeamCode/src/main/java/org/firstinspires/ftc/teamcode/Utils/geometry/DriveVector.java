package org.firstinspires.ftc.teamcode.Utils.geometry;

public class DriveVector {
    private double x,y,z;

    public DriveVector(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public DriveVector(double x, double y){
        this(x,y,0);
    }

    public DriveVector(){
        this(0,0,0);
    }

    public DriveVector(DriveVector otherVector){
        this.x = otherVector.x;
        this.y = otherVector.y;
        this.z = otherVector.z;
    }

    public static DriveVector fromAngleAndMagnitude(double t, double m){
        return new DriveVector(m*Math.cos(t), m*Math.sin(t));
    }

    //t1 is angle in xy plane, t2 is angle with xy plane
    public static DriveVector fromAngleAndMagnitude(double t1, double t2, double m){
        return new DriveVector(Math.cos(t1)*Math.cos(t2)*m, Math.sin(t1)*Math.cos(t2)*m, Math.sin(t2)*m);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getMagnitude(){
        return Math.sqrt(x*x + y*y + z*z);
    }

    public DriveVector plus(DriveVector other){
        return new DriveVector(x + other.getX(), y + other.getY(), z + other.getZ());
    }

    public void scaleToMagnitude(double targetMagnitude){
        double currentMagnitude = getMagnitude();
        scaleBy(1.0/currentMagnitude);
        scaleBy(targetMagnitude);
    }

    public void scaleBy(double a){
        x = x * a;
        y = y * a;
        z = z * a;
    }

    public DriveVector scaledBy(double a){
        return new DriveVector(x * a, y * a, z * a);
    }

    public DriveVector scaledToMagnitude(double targetMagnitude){
        DriveVector aux = new DriveVector(this);
        aux.scaleToMagnitude(targetMagnitude);
        return aux;
    }

    public static DriveVector rotateBy(DriveVector vector, double theta){
        return new DriveVector(Math.cos(theta) * vector.getX() + Math.sin(theta) * vector.getY(), Math.cos(theta) * vector.getY() - Math.sin(theta) * vector.getX());
    }

    @Override
    public String toString(){
        return String.valueOf(x) + " " + String.valueOf(y) + " " + String.valueOf(z);
    }
}
