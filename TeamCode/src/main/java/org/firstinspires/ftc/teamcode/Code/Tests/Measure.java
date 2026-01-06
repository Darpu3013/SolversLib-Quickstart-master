package org.firstinspires.ftc.teamcode.Code.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Measure")
public class Measure extends OpMode {

    private int pathState;
    private Follower follower;
    private Timer pathTimer;


    private final Pose startPose = new Pose(38.5, 20, Math.toRadians(90));
    private final Pose goingPose  = new Pose(38.5, 60, Math.toRadians(90));


    private PathChain straightLine, turn;

    public void buildPaths() {

        straightLine = follower.pathBuilder()
                .addPath(new BezierLine(startPose, goingPose))
                .setConstantHeadingInterpolation(Math.toRadians((90)))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(straightLine);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                // End of autonomous — do nothing
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);

        buildPaths();
    }

    @Override
    public void start() {
        setPathState(0);
    }
}
