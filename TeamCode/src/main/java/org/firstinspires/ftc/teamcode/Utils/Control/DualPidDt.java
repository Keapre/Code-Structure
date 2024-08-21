package org.firstinspires.ftc.teamcode.Utils.Control;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class DualPidDt {


    public final PIDController aggpid;
    public final PIDController chillpid;
    public PIDController currentPid;
    double integralSumLimit = 0;
    public double target;
    public double lastTarget;

    double error= 0;
    double integral = 0;
    long lastLoopTime = System.nanoTime();
    double lastError = 0;
    int counter = 0;

    public static double tolerance = 3;
    double loopTime = 0;
    double threeshold = 0;
    public DualPidDt(PIDCoefficients agressiveCoeffiecent,PIDCoefficients chillCoeffiecent) {
        aggpid = new PIDController(agressiveCoeffiecent.p,agressiveCoeffiecent.i,agressiveCoeffiecent.d);
        chillpid = new PIDController(chillCoeffiecent.p,chillCoeffiecent.i,chillCoeffiecent.d);
    }

    public double getThreeshold() {
        return threeshold;
    }

    public double getTolerance() {
        return tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }
    public void setThreeshold() {
        this.threeshold = threeshold;
    }

    public void getTargetValue(double value) {
        this.target = value;
    }

    public double calculate(double current) {
        if(lastTarget!=target) {
            integral = 0;
        }
        double error = target - current;

        if(error <= tolerance) return 0;
        if(error < threeshold) currentPid = chillpid;
        else currentPid = aggpid;

        currentPid.setSetPoint(target);
        return currentPid.calculate(current);
    }

}
