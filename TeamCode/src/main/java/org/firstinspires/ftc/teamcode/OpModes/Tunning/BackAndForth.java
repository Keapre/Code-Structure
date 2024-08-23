package org.firstinspires.ftc.teamcode.OpModes.Tunning;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.LogicNode;
import org.firstinspires.ftc.teamcode.Subsystem.Drive.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystem.Sensors;
import org.firstinspires.ftc.teamcode.Utils.geometry.Pose;

public class BackAndForth extends LinearOpMode {

    DriveTrain dt = null;
    Sensors sensors = null;
    private LogicNode current = new LogicNode("Start");
    private LogicNode forward = new LogicNode("go forward");
    private LogicNode back = new LogicNode("go backward");

    private Pose ForwardPose = new Pose(5,0,0);
    private Pose BackwardPose = new Pose(-5,0,0);
    @Override
    public void runOpMode() throws InterruptedException {
        sensors = new Sensors(this.hardwareMap);
        dt = new DriveTrain(hardwareMap,new Pose(0,0,0),sensors);

        initNodes();
        dt.runMode = DriveTrain.RunMode.PID;

        waitForStart();
        while(opModeIsActive()) {
            current.run();
            dt.update();

            telemetry.addData("Node", current);
            telemetry.addData("Pose", dt.poseUpdater.getPose());
            telemetry.update();
        }
    }

    public void initNodes() {
        current.addCondition(
                () -> true,
                () -> {
                    dt.setTargetPose(ForwardPose);
                },forward
        );

        forward.addCondition(
                () -> (dt.reachedTarget(1)),
                () -> {
                    dt.setTargetPose(BackwardPose);
                },back
        );

        back.addCondition(
                () -> (dt.reachedTarget(1)),
                () -> {
                    dt.setTargetPose(ForwardPose);
                },forward
        );
    }
}
