package org.firstinspires.ftc.teamcode.Subsystem.Drive;


import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;



import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import org.firstinspires.ftc.teamcode.Subsystem.Drive.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.Subsystem.Sensors;
import org.firstinspires.ftc.teamcode.Utils.Control.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.Utils.Control.DualPidDt;
import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.SmoothMotor;
import org.firstinspires.ftc.teamcode.Utils.geometry.DriveVector;
import org.firstinspires.ftc.teamcode.Utils.geometry.Pose;
import org.firstinspires.ftc.teamcode.Utils.geometry.PoseUpdater;
import org.firstinspires.ftc.teamcode.Utils.geometry.Vector;
import org.firstinspires.ftc.teamcode.Utils.pubSub.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.Control.FilteredPIDFController;

import java.util.Arrays;
import java.util.List;


public class DriveTrain extends Subsystem {

    Pose start;
    SmoothMotor leftFront,rightFront,leftBack,rightBack;

    private final List<SmoothMotor> motors;
    public DriveVector powerVector = new DriveVector();
    public static double lateralMultiplier = 2;

    public static double headingMultiplier = 1;
    private Pose targetPose;
    public DriveVector targetVector = new DriveVector();
    public PoseUpdater poseUpdater;

    public double xThreeshold = 1,yTreeshold = 1,headingThreeshold = 5;
    public double finalxThreeshold = 0.5,finalyTreeshold = 0.5,finalheadingThreeshold = 2.5;

    public static double ks = 0.02;
    double xError = 0;
    double yError = 0;
    double turnError = 0;
    // public static CustomFilteredPIDFCoefficients agressiveTranslationalPID = new CustomFilteredPIDFCoefficients(0,0,0,0.5,0),
    //         chillTranslationalPID = new CustomFilteredPIDFCoefficients(0,0,0,0.5,0);

    // public static DualPidDt translationalPID = new DualPidDt(agressiveTranslationalPID,chillTranslationalPID);
    // public static CustomFilteredPIDFCoefficients agressiveHeadingnPID = new CustomFilteredPIDFCoefficients(0,0,0,0.5,0),chillHeadingPID = new CustomFilteredPIDFCoefficients(0,0,0,0.5,0);

    //public static DualPidDt headingPID = new DualPidDt(agressiveHeadingnPID,chillHeadingPID);

    public static CustomFilteredPIDFCoefficients xPidCof = new CustomFilteredPIDFCoefficients(0,0,0,0.5,0),yPidCof = new CustomFilteredPIDFCoefficients(0,0,0,0.5,0),headingPidCof = new CustomFilteredPIDFCoefficients(0,0,0,0.5,0);
    long perfectHeadingTimeStart = System.currentTimeMillis();
    public static FilteredPIDFController xPid = new FilteredPIDFController(xPidCof),
    yPid = new FilteredPIDFController(yPidCof),
    headingPid = new FilteredPIDFController(headingPidCof);
    public static PIDFController finalXPID = new PIDFController(0.035, 0.0,0.0,0);
    public static PIDFController finalYPID = new PIDFController(0.1, 0.0,0.0,0);
    public static PIDFController finalTurnPID = new PIDFController(0.01, 0.0,0.0,0);
    public enum RunMode {
        MANUAL,
        PID,
        BRAKE,
        FINAL_ADJUSTEMENT
    }

    // leftFront, leftRear, rightRear, rightFront
    double[] minPowersToOvercomeFriction = new double[] {
            0.3121803239920063,
            0.3533249418072871,
            0.36038420175052865,
            0.39695077434023707
    };
    public RunMode runMode;
    Sensors sensors;

    double tolerancePoint = 2;
    public DriveTrain(HardwareMap hardwareMap, Pose start, Sensors sensors) {
        this.sensors = sensors;
        leftFront = new SmoothMotor(hardwareMap.get(DcMotorEx.class,"leftFront"),sensors);
        leftBack = new SmoothMotor(hardwareMap.get(DcMotorEx.class,"leftBack"),sensors);
        rightBack = new SmoothMotor(hardwareMap.get(DcMotorEx.class,"rightBack"),sensors);
        rightFront = new SmoothMotor(hardwareMap.get(DcMotorEx.class,"rightFront"),sensors);

        motors = Arrays.asList(leftBack,leftFront,rightFront,rightBack);

        setModeMotors();
        runMode = RunMode.MANUAL;
        this.start = start;
        poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(start);

    }


    public boolean reachedTarget() {
        if(runMode == RunMode.BRAKE) return true;
        return (xError < xThreeshold && yError < yTreeshold) || Utils.calculateDistanceBetweenPoints(poseUpdater.getPose(),targetPose) < tolerancePoint;
    }

    public boolean reachedHeading(){
        if(runMode == RunMode.BRAKE) return true;

        return Math.abs(turnError) < headingThreeshold;
    }

    boolean finalAdjustment = false;
    public void setModeMotors() {

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void update() {
        updateState();
        updateMotors();
        if(runMode == RunMode.PID) this.UPDATABLE = true;
    }

    public void goToPoint(Pose targetPose,boolean adjustement){
        this.finalAdjustment = adjustement;
        setTargetPose(targetPose);
        runMode = RunMode.PID;
    }
    public void updateMotors(){

        double voltage = sensors.getVoltage();
        double actualKs = ks * 12.0 / voltage;

        leftFront.setTargetPower((powerVector.getX() - powerVector.getY() - powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() - powerVector.getY() - powerVector.getZ()));
        rightFront.setTargetPower((powerVector.getX() + powerVector.getY() + powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() + powerVector.getY() + powerVector.getZ()));
        leftBack.setTargetPower((powerVector.getX() + powerVector.getY() - powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() + powerVector.getY() - powerVector.getZ()));
        rightBack.setTargetPower((powerVector.getX() - powerVector.getY() + powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() - powerVector.getY() + powerVector.getZ()));
    }


    public void updateState() {

        calculateErrors();
        poseUpdater.update();
        switch (runMode){
            case MANUAL:
                powerVector = new DriveVector(targetVector.getX(), targetVector.getY(), targetVector.getZ());
                powerVector = DriveVector.rotateBy(powerVector, poseUpdater.getPose().getHeading());
                powerVector = new DriveVector(powerVector.getX(), powerVector.getY() * lateralMultiplier, targetVector.getZ());
                break;
            case PID:
                setMinPowersToOvercomeFriction();
                PIDF();
                if(reachedTarget()) {
                    if(finalAdjustment) {
                        finalTurnPID.clearTotalError();
                        perfectHeadingTimeStart = System.currentTimeMillis();
                        runMode = RunMode.FINAL_ADJUSTEMENT;
                    }else {
                        runMode = RunMode.BRAKE;
                    }
                }
                break;
            case FINAL_ADJUSTEMENT:
                finalAdjustment();
                if (Math.abs(turnError) < Math.toRadians(finalheadingThreeshold)) {
                    if (System.currentTimeMillis() - perfectHeadingTimeStart > 150) {
                        runMode = RunMode.BRAKE;
                    }
                } else {
                    perfectHeadingTimeStart = System.currentTimeMillis();
                }
                break;
            case BRAKE:
                powerVector = new DriveVector(0,0,0);
                break;
            default:
                break;
        }

    }
    public void calculateErrors() {
        double deltaX = targetPose.getX() - poseUpdater.getPose().getX();
        double deltaY = targetPose.getY() - poseUpdater.getPose().getY();

        xError = deltaX * Math.cos(poseUpdater.getPose().getHeading()) + deltaY * Math.sin(poseUpdater.getPose().getHeading());
        yError = deltaY * Math.cos(poseUpdater.getPose().getHeading()) - deltaX * Math.sin(poseUpdater.getPose().getHeading());
        turnError = targetPose.getHeading() - poseUpdater.getPose().getHeading();

        while(Math.abs(turnError) > Math.PI ){
            turnError -= Math.PI * 2 * Math.signum(turnError);
        }
    }
    public void PIDF() {
        double globalXError = targetPose.getX() - poseUpdater.getPose().getX();
        double globalYError = targetPose.getY() - poseUpdater.getPose().getY();

        double localXError = globalXError * Math.cos(poseUpdater.getPose().getHeading()) + globalYError * Math.sin(poseUpdater.getPose().getHeading());
        double localYError = globalYError * Math.cos(poseUpdater.getPose().getHeading()) - globalXError * Math.sin(poseUpdater.getPose().getHeading());

        double distanceToPoint = Utils.calculateDistanceBetweenPoints(poseUpdater.getPose(), targetPose);

        double fwd,strafe,turn;
        if(Math.abs(distanceToPoint) < 20) {
            xPid.updatePosition(localXError);
            yPid.updatePosition(localYError);
            fwd = Math.abs(localXError) > xThreeshold/2 ? Utils.minMaxClip(xPid.runPIDF(),-1,1) + 0.05 * Math.signum(localXError) : 0;
            strafe = Math.abs(localYError) > yTreeshold/2 ? Utils.minMaxClip(yPid.runPIDF(),-1,1) + 0.05 * Math.signum(localYError) : 0;
        }else {
            fwd = Math.abs(xError) > xThreeshold/2 ? Utils.minMaxClip(xPid.runPIDF(),-1,1) + 0.05 * Math.signum(xError) : 0;
            strafe = Math.abs(yError) > yTreeshold/2 ? Utils.minMaxClip(yPid.runPIDF(),-1,1) + 0.05 * Math.signum(yError) : 0;
        }

        double turnAdjustThreeshold = (Math.abs(xError) > xThreeshold/2 || Math.abs(yError) > yTreeshold/2 ) ? headingThreeshold/3.0 : headingThreeshold;
        headingPid.updatePosition(turnError);
        turn = Math.abs(turnError) > Math.toRadians(turnAdjustThreeshold) ? Utils.minMaxClip(headingPid.runPIDF(),-1,1) : 0;

        powerVector = new DriveVector(fwd,strafe,turn);
    }

    public void finalAdjustment() {
        double fwd = Math.abs(xError) > finalxThreeshold/2 ? Utils.minMaxClip(finalXPID.calculate(xError),-1,1)  : 0;
        double strafe = Math.abs(yError) > finalyTreeshold/2 ? Utils.minMaxClip(finalYPID.calculate(yError),-1,1) : 0;
        double turn = Math.abs(turnError) > Math.toRadians(finalheadingThreeshold)/2 ? Utils.minMaxClip(finalTurnPID.calculate(turnError),-1,1) : 0;

        powerVector = new DriveVector(fwd,strafe,turn);
    }

    public void setMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[0]);
        leftBack.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[1]);
        leftBack.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[2]);
        rightFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[3]);
        for (SmoothMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0.195);
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (SmoothMotor motor : motors) {
            motor.setMode(runMode);
        }
    }
    public void resetMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeStaticFriction(0.0);
        leftBack.setMinimumPowerToOvercomeStaticFriction(0.0);
        rightBack.setMinimumPowerToOvercomeStaticFriction(0.0);
        rightFront.setMinimumPowerToOvercomeStaticFriction(0.0);
        for (SmoothMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0);
        }
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (SmoothMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        leftFront.setTargetPower(lf);
        leftBack.setTargetPower(lr);
        rightBack.setTargetPower(rr);
        rightFront.setTargetPower(rf);
    }


    public void setTargetPose(Pose pose){
        this.targetPose = pose;
    }

    public void setTargetVector(DriveVector Vector){
        this.targetVector = Vector;
    }

    public RunMode getRunMode() {
        return runMode;
    }



    public Pose getTargetPose(){
        return targetPose;
    }

    public void setRunMode(RunMode runMode){
        this.runMode = runMode;
    }
}
