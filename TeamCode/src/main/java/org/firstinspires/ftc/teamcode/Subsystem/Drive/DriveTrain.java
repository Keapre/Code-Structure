package org.firstinspires.ftc.teamcode.Subsystem.Drive;

import com.acmerobotics.dashboard.canvas.Canvas;

import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.Subsystem.Drive.Localizer.BetterLocalizer;
import org.firstinspires.ftc.teamcode.Subsystem.Sensors;
import org.firstinspires.ftc.teamcode.Utils.Control.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.Utils.Dashboard.TelemetryUtil;
import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.SmoothMotor;
import org.firstinspires.ftc.teamcode.Utils.geometry.DriveVector;
import org.firstinspires.ftc.teamcode.Utils.geometry.Pose;
import org.firstinspires.ftc.teamcode.Utils.geometry.PoseUpdater;
import org.firstinspires.ftc.teamcode.Utils.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.Utils.pubSub.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.Control.FilteredPIDFController;

import java.lang.Math;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;


@SuppressWarnings("unused")
public class DriveTrain extends Subsystem {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        public double inPerTick = 1;
        public double lateralInPerTick = inPerTick;
        public double trackWidthTicks = 0;
    }

    Pose2d start;
    public Pose2d pose;
    public final LazyImu lazyImu;
    BetterLocalizer betterLocalizer;
    SmoothMotor leftFront,rightFront,leftBack,rightBack;

    private final List<SmoothMotor> motors;
    public DriveVector powerVector = new DriveVector();
    public static double lateralMultiplier = 2;

    public static double headingMultiplier = 1;
    private Pose2d targetPose;
    public DriveVector targetVector = new DriveVector();
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
    public final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public static PIDFController finalXPID = new PIDFController(0.035, 0.0,0.0,0);
    public static PIDFController finalYPID = new PIDFController(0.1, 0.0,0.0,0);
    public static PIDFController finalTurnPID = new PIDFController(0.01, 0.0,0.0,0);
    public enum RunMode {
        MANUAL,
        PID,
        BRAKE,
        GO_TO_POINT,
        FINAL_ADJUSTEMENT,
        WAIT_AT_POINT
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
    public static Params PARAMS = new Params();
    Pose2d poseWithoutBacktracking = new Pose2d(0.0, 0.0, 0.0);
    double[] OverallError = new double[3];

    double tolerancePoint = 1.5;
    boolean transvitive = false;
    public DriveTrain(HardwareMap hardwareMap, Pose2d start, Sensors sensors) {
        this.sensors = sensors;
        leftFront = new SmoothMotor(hardwareMap.get(DcMotorEx.class,"leftFront"),sensors);
        leftBack = new SmoothMotor(hardwareMap.get(DcMotorEx.class,"leftBack"),sensors);
        rightBack = new SmoothMotor(hardwareMap.get(DcMotorEx.class,"rightBack"),sensors);
        rightFront = new SmoothMotor(hardwareMap.get(DcMotorEx.class,"rightFront"),sensors);

        motors = Arrays.asList(leftBack,leftFront,rightFront,rightBack);

        setModeMotors();
        runMode = RunMode.MANUAL;
        this.start = start;
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        pose = start;
        betterLocalizer = new BetterLocalizer(hardwareMap,lazyImu.get(),PARAMS.inPerTick,this);


    }


    public boolean reachedTarget(){
        if(runMode == RunMode.BRAKE) return true;

        if(Utils.calculateDistanceBetweenPoints(pose,targetPose) <= tolerancePoint) {
            return true;
        }

        if(finalAdjustment && runMode != RunMode.PID) {
            return Math.abs(xError) < xThreeshold && Math.abs(yError) < yTreeshold && Math.abs(turnError) < Math.toRadians(finalheadingThreeshold);
        }

        if(transvitive) {
            //putem sa avem restrici mai mici pentru x si y
            return Math.abs(xError) < xThreeshold * 3  && Math.abs(yError) < yTreeshold * 3 && Math.abs(turnError) < Math.toRadians(headingThreeshold);
        }

        return Math.abs(xError) < xThreeshold && Math.abs(yError) < yTreeshold && Math.abs(turnError) < Math.toRadians(headingThreeshold);
    }

    public boolean atPointThresholds (double xThresh, double yThresh, double headingThresh) {
        return Math.abs(xError) < xThresh && Math.abs(yError) < yThresh && Math.abs(turnError) < Math.toRadians(headingThresh);
    }
//    public boolean reachedHeading(){
//        if(runMode == RunMode.BRAKE) return true;
//
//        return Math.abs(turnError) < headingThreeshold;
//    }

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

    public void goToPoint(Pose2d targetPose,boolean adjustement,boolean transitive){
        this.finalAdjustment = adjustement;
        this.transvitive = transitive;
        setTargetPose(targetPose);
        runMode = RunMode.PID;
    }
    public void updateMotors(){

        double voltage = sensors.getVoltage();
        double actualKs = ks * 12.0 / voltage;

        leftFront.setTargetPower((powerVector.getX() - powerVector.getY() - powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() - powerVector.getY() - powerVector.getZ()));
        rightFront.setTargetPower((powerVector.getX() + powerVector.getY() + powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() + powerVector.getY() + powerVector.getZ()));
        leftBack.setTargetPower((powerVector.getX() + powerVector.getY()- powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX()+ powerVector.getY()- powerVector.getZ()));
        rightBack.setTargetPower((powerVector.getX()- powerVector.getY()+ powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() - powerVector.getY() + powerVector.getZ()));
    }


    public void updateState() {
        updatePoseEstimate();
        calculateErrors();
        updateTelemetry();
        switch (runMode){
            case MANUAL:
                powerVector = new DriveVector(targetVector.getX(), targetVector.getY(), targetVector.getZ());
                powerVector = DriveVector.rotateBy(powerVector, pose.heading.toDouble());
                powerVector = new DriveVector(powerVector.getX(), powerVector.getY() * lateralMultiplier, targetVector.getZ());
                break;
            case GO_TO_POINT:
                
                break;
            case PID:
                setMinPowersToOvercomeFriction();
                PIDF();
                if(reachedTarget()){
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
                runMode = RunMode.WAIT_AT_POINT;
                break;
            case WAIT_AT_POINT:
                if(atPointThresholds(1.5,1.5,1.5)) {
                    resetIntegrals();
                    runMode = RunMode.PID;
                }
            default:
                break;
        }

    }

    public void resetIntegrals() {
        xPid.resetIntegral();
        yPid.resetIntegral();
        headingPid.resetIntegral();
    }
    public void calculateErrors() {
        double deltaX = targetPose.position.x - pose.position.x;
        double deltaY = targetPose.position.y - pose.position.y;

        xError = deltaX * Math.cos(pose.heading.toDouble()) + deltaY * Math.sin(pose.heading.toDouble());
        yError = deltaY * Math.cos(pose.heading.toDouble()) - deltaX * Math.sin(pose.heading.toDouble());
        turnError = targetPose.heading.toDouble() - pose.heading.toDouble();

        while(Math.abs(turnError) > Math.PI ){
            turnError -= Math.PI * 2 * Math.signum(turnError);
        }
    }
    public double smoothControls(double value) {
        return 0.5*Math.tan(1.12*value);
    }
    public void drive(Gamepad gamepad) {
        resetMinPowersToOvercomeFriction();
        runMode = RunMode.MANUAL;

        double forward = smoothControls(gamepad.left_stick_y);
        double strafe = smoothControls(gamepad.left_stick_x);
        double turn = smoothControls(-gamepad.right_stick_x);

        DriveVector drive = new DriveVector(forward,strafe);
        if (drive.getMagnitude() <= 0.05){
            drive.scaleBy(0);
        }
        drive = new DriveVector(pose.position.x,pose.position.y,turn);
    }
    public boolean isBusy() {
        return runMode != RunMode.WAIT_AT_POINT;
    }
    public void updateTelemetry () {
        TelemetryUtil.packet.put("Drivetrain State", runMode);

        TelemetryUtil.packet.put("xError", xError);
        TelemetryUtil.packet.put("yError", yError);
        TelemetryUtil.packet.put("turnError (deg)", Math.toDegrees(turnError));

//        TelemetryUtil.packet.put("maxPower", maxPower);

        TelemetryUtil.packet.fieldOverlay().setStroke("red");
        TelemetryUtil.packet.fieldOverlay().strokeCircle(targetPose.position.x, targetPose.position.y, xThreeshold);

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        canvas.setStroke("blue");
        canvas.strokeLine(pose.position.x, pose.position.y, targetPose.position.x, targetPose.position.y);
    }

    private double lastDrift = 0.0;
    public static double angleError = 0.0;

    public void correctCurrentPose(ArrayList<ArrayList<Double>> changesInPos, ArrayList<Pose2d> posHistory, ArrayList<Twist2d> twistChanges, double totalAngleDrift) {
        angleError += (totalAngleDrift - lastDrift);

        //loop through each estimated pose
        for (int i = 0; i < changesInPos.size(); i++) {
            double timeDrift = 1.0 / changesInPos.size() * totalAngleDrift;
            correctThePose(
                    new double[]{
                            changesInPos.get(i).get(0),
                            changesInPos.get(i).get(1),
                            changesInPos.get(i).get(2) + timeDrift},

                    posHistory.get(i),
                    twistChanges.get(i));
        }

        changesInPos.clear();
        posHistory.clear();
        twistChanges.clear();

        lastDrift = angleError;
    }


    private double[] lastErrors = new double[3];
    private double[] errors = new double[2];

    public void correctThePose(double[] change, Pose2d prevPose, Twist2d estTwist) {
        Twist2dDual<Time> imuTwist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[]{
                                change[0],
                                0.0,
                        }).times(PARAMS.inPerTick),
                        new DualNum<Time>(new double[]{
                                change[1],
                                0.0,
                        }).times(PARAMS.inPerTick)
                ),
                new DualNum<>(new double[]{
                        change[2],
                        0.0,
                })
        );

        //because RR does not support subtracting twists or poses

        //Find pose using odo twist
        Pose2d previousPose = prevPose;
        previousPose = previousPose.plus(estTwist);
        double[] prevPosePoses = new double[]{previousPose.position.x, previousPose.position.y};

        //Find pose using imu twist
        Pose2d newPose = prevPose;
        newPose = newPose.plus(imuTwist.value());
        double[] newPosePoses = new double[]{newPose.position.x, newPose.position.y};

        for (int i = 0; i < 2; i++) {
            errors[i] += newPosePoses[i] - prevPosePoses[i];
        }

        pose = new Pose2d(
                pose.position.x + errors[0] - lastErrors[0],
                pose.position.y + errors[1] - lastErrors[1],
                (pose.heading.plus(angleError - lastErrors[2])).toDouble()
        );

        lastErrors = new double[]{
                errors[0],
                errors[1],
                angleError
        };
    }
    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = betterLocalizer.update();
        pose = pose.plus(twist.value());

        poseWithoutBacktracking = poseWithoutBacktracking.plus(twist.value());

        OverallError = new double[]{
                Math.abs(poseWithoutBacktracking.position.x - pose.position.x),
                Math.abs(poseWithoutBacktracking.position.y - pose.position.y),
                Math.abs(poseWithoutBacktracking.heading.minus(pose.heading))
        };

        poseHistory.add(pose);

        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        //DW return value never used
        return twist.velocity().value();
    }
    double[] percentage_Correction = new double[2], prevPose = new double[2];
    private ArrayList<Double> totalChange = new ArrayList<>(Arrays.asList(0.0, 0.0));


    // This returns the effectiveness of the Localizer in percentage
    public double[] totalMovement() {
        double[] currentPose = new double[]{pose.position.x, pose.position.y};
        double[] difference = new double[2];
        for (int i = 0; i < 2; i++) {
            difference[i] = Math.abs(currentPose[i] - prevPose[i]);
            totalChange.set(i, totalChange.get(i) + difference[i]);
            if (!totalChange.contains(0.0)) {
                percentage_Correction[i] = 100 * Math.abs(OverallError[i]) / totalChange.get(i);
            }
        }
        prevPose = currentPose;
        return percentage_Correction;
    }
    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public void PIDF() {
        double globalXError = targetPose.position.x - pose.position.x;
        double globalYError = targetPose.position.y - pose.position.y;

        double localXError = globalXError * Math.cos(pose.heading.toDouble()) + globalYError * Math.sin(pose.heading.toDouble());
        double localYError = globalYError * Math.cos(pose.heading.toDouble()) - globalXError * Math.sin(pose.heading.toDouble());

        double distanceToPoint = Utils.calculateDistanceBetweenPoints(pose, targetPose);

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


    public void setTargetPose(Pose2d pose){
        this.targetPose = pose;
    }

    public void setTargetVector(DriveVector Vector){
        this.targetVector = Vector;
    }

    public RunMode getRunMode() {
        return runMode;
    }



    public Pose2d getTargetPose(){
        return targetPose;
    }

    public void setRunMode(RunMode runMode){
        this.runMode = runMode;
    }
}
