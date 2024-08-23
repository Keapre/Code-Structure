package org.firstinspires.ftc.teamcode.Utils.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.Subsystem.Sensors;
import org.firstinspires.ftc.teamcode.Utils.Utils;

public class SmoothMotor {
    DcMotorEx motor;
    private long lastZeroTime = 0;

    public double power = 0;
    public double lastPower = 0;


    private double minPowerToOvercomeStaticFriction = 0.0;
    public static final int SWITCH_FROM_STATIC_TO_KINETIC_FRICTION = 75;
    private double minPowerToOvercomeKineticFriction = 0.0;

    Sensors sensors;
    public SmoothMotor(DcMotorEx mtr, Sensors sensors) {
        motor = mtr;

        power = lastPower = 0;
        this.sensors = sensors;

    }
    public void setMinimumPowerToOvercomeStaticFriction (double value) {
        minPowerToOvercomeStaticFriction = value;
    }

    public void setMinimumPowerToOvercomeKineticFriction(double value) {
        minPowerToOvercomeKineticFriction = value;
    }
    public void setTargetPower(double power) {
        if (lastPower == 0) {
            lastZeroTime = System.currentTimeMillis();
        }
        power = Utils.minMaxClip(power, -1.0, 1.0);
        double m = 0;
        m = (System.currentTimeMillis() > SWITCH_FROM_STATIC_TO_KINETIC_FRICTION + lastZeroTime ? minPowerToOvercomeKineticFriction : minPowerToOvercomeStaticFriction) * (12/sensors.getVoltage());
        power *= 1-m;
        this.power = power + m * Math.signum(power);

        if(lastPower != power) {
            motor.setPower(power);
        }
        lastPower = power;
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behaviour) {
        motor.setZeroPowerBehavior(behaviour);
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public void setDirection(DcMotorSimple.Direction dir){
        motor.setDirection(dir);
    }
}

