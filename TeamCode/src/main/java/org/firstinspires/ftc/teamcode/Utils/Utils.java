package org.firstinspires.ftc.teamcode.Utils;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Utils.geometry.Pose;

public class Utils {
    public Utils() {}

    public static double minMaxClip(double value, double min, double max) {
        return Math.min(Math.max(min, value), max);
    }

    public static int minMaxClipInt(double value, double min, double max) {
        return (int) Math.min(Math.max(min, value), max);
    }

    public static double headingClip(double value) {
        while(value >= Math.PI) {
            value -= 2*Math.PI;
        }
        while(value <= -Math.PI) {
            value += 2*Math.PI;
        }
        return value;
    }

    public static boolean withinThreshold(double value, double minThreshold, double maxThreshold) {
        return value > minThreshold && value < maxThreshold;
    }

    public static double kalmanFilter (double value1, double value2, double value2Weight) {
        return (value1 * (1.0-value2Weight)) + (value2 * value2Weight);
    }

    public static double calculateDistanceBetweenPoints (Pose point1, Pose point2) {
        double deltaX = point1.getX() - point2.getX() ;
        double deltaY = point1.getY()  - point2.getY() ;
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }
}
