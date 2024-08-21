package org.firstinspires.ftc.teamcode.Utils.Wrappers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Eimu {
    public final IMU imu;
    public static double imuAngle = 0;
    public static double imuVelocity = 0;
    public static double imuOffset = 0;

    public Eimu(HardwareMap hardwareMap) {

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN
        ));
        imu.initialize(parameters);

        imu.resetYaw();
    }


    public synchronized void resetImu(){
        imu.resetDeviceConfigurationForOpMode();
        imuOffset = 0;
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN
        ));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void update(){
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getHeading() {
        update();
        return imuAngle + imuOffset;
    }
    public double getVelocity() {
        update();
        return imuVelocity;
    }


}
