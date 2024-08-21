package org.firstinspires.ftc.teamcode.Utils.Control;

public class LowFilter {
    private final double t;
    private double lastValue;

    public LowFilter(double t, double initialValue){
        this.t = t;
        this.lastValue = initialValue;
    }

    public double getValue(double rawValue){
        double newValue = lastValue + t * (rawValue - lastValue);
        this.lastValue = newValue;
        return newValue;
    }
}
