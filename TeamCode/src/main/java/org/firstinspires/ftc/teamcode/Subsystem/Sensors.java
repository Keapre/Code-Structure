package org.firstinspires.ftc.teamcode.Subsystem;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.teamcode.Utils.pubSub.Subsystem;

import java.util.List;

public class Sensors extends Subsystem {
    private List<LynxModule> allHubs;

    private double leftFrontMotorCurrent, leftRearMotorCurrent, rightRearMotorCurrent, rightFrontMotorCurrent;
    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
    private double imuUpdateRate = 1500;

//    HuskyLens huskyLens;
//    HuskyLens.Block[] huskyLensBlocks;

    private long lastImuUpdateTime = System.currentTimeMillis();
    private IMU imu;

    private long imuLastUpdateTime = System.currentTimeMillis();
    private double imuHeading = 0.0;
    public boolean useIMU;
    public int numPixels = 0;


    HardwareMap hardwareMap;
    public double extendoEncoder, extendoVelocity;
    public double slideEncoder, slideVelocity;
    DigitalChannel intakePixelLeft;
    DigitalChannel intakePixelRight;
    private double voltage;

    public static double updateTimeVoltage = 5000;
    long lastVoltageUpdatedTime = System.currentTimeMillis();

    private static final double pixelThreeshold = 5; // MM
    private double imuUpdateTime = 800;
    public double timeTillNextIMUUpdate = imuUpdateTime;
    public boolean imuJustUpdated = false;

    private double huskyUpdateTime = 1400;

    long lastHuskyLensUpdatedTime = System.currentTimeMillis();
    public boolean huskyJustUpdated = false;

    public boolean huskyLensEnabled = false;

    public Sensors(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        intakePixelLeft = hardwareMap.get(DigitalChannel.class, "poluluLeft");
        intakePixelRight = hardwareMap.get(DigitalChannel.class, "poluluRight");

        intakePixelLeft.setMode(DigitalChannel.Mode.INPUT);
        intakePixelRight.setMode(DigitalChannel.Mode.INPUT);
        //this.robot = robot;
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub :allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        initSensors(hardwareMap);
    }

    void initSensors(HardwareMap hardwareMap) {

//        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
//        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        //imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(
//                new IMU.Parameters(new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//                )
//                )
//        );
//        imu.resetYaw();
//        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
//        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//

        //voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void update() {
        updateControlHub();
        //expansionHubUpdate();
    }



    public void updateControlHub() {
        imuJustUpdated = false;
        huskyJustUpdated = false;
        long currTime = System.currentTimeMillis();

        numPixels = pixelCounter();
//        if (useIMU && currTime - imuLastUpdateTime >= imuUpdateTime) {
//            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//            imuHeading = orientation.getYaw(AngleUnit.RADIANS);
//            imuLastUpdateTime = currTime;
//            imuJustUpdated = true;
//        }

/*        if(huskyLensEnabled && currTime - lastHuskyLensUpdatedTime >= huskyUpdateTime) {
            huskyLensBlocks = huskyLens.blocks();
            lastHuskyLensUpdatedTime = currTime;
            huskyJustUpdated = true;
        }*/

        if (currTime- lastVoltageUpdatedTime > updateTimeVoltage) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = currTime;
        }
        timeTillNextIMUUpdate = imuUpdateTime - (currTime - imuLastUpdateTime);
/*        extendoEncoder = ((PriorityMotor) hardwareQueue.getDevice("extendo")).motor[0].getCurrentPosition(); //TODO : daca encoderu e reversed
        extendoVelocity = ((PriorityMotor) hardwareQueue.getDevice("extendo")).motor[0].getVelocity(); //SI AICI LA FEL

        slideEncoder = ((PriorityMotor) hardwareQueue.getDevice("slides1")).motor[0].getCurrentPosition();
        slideVelocity = ((PriorityMotor) hardwareQueue.getDevice("slides1")).motor[0].getVelocity();*/
    }

    private void expansionHubUpdate() {
        try {

        } catch (Exception e) {
            Log.e("******* Error due to", e.getClass().getName());
            e.printStackTrace();
            Log.e("***** fail to update", "Expansion Hub");
        }
    }

    public int pixelCounter() {
        int cnt = 0;
        if(!getLeftDistance()) cnt++;
        if(getRightDistance()) cnt++;

        return cnt;
    }

    private double previousAngle = 0.0;
    private int numRotations = 0;
    private void addToCumulativeHeading(double angle) {
        if (Math.abs(angle-previousAngle) >= Math.toRadians(180)) {
            numRotations += Math.signum(previousAngle);
        }
        previousAngle = angle;
    }
    public double getImuHeading() {
        return imuHeading;
    }

    public boolean getLeftDistance() {
        return intakePixelLeft.getState();
    }

    public boolean getRightDistance() {
        return intakePixelRight.getState();
    }
    public double getExtendoPos() {
        return extendoEncoder;
    }
    public double getExtendoVelocity() {
        return extendoVelocity;
    }

//    public HuskyLens.Block[] getHuskyLensBlocks() {
//        return huskyLensBlocks;
//    }

    public double getNormalizedIMUHeading() {
        return getImuHeading() - (numRotations*(2*Math.PI));
    }
    public double getSlidePos() {
        return slideEncoder;
    }
    public double getSlideVelocity() {
        return slideVelocity;
    }
    public double getVoltage() {
        return voltage;
    }
}
