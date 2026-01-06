package org.firstinspires.ftc.teamcode.Code.Real;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueAuto_Ethan_Half")
public class BlueAuto_Ethan_Half extends OpMode {

    private int pathState;
    private Follower follower;
    private Timer pathTimer;

    private final Pose startPose = new Pose(64, 9, Math.toRadians(90));
    private final Pose goingToPick = new Pose(59, 45, Math.toRadians(180));
    private final Pose firstBatch = new Pose(20, 45, Math.toRadians(180));

    private final Pose shooting = new Pose(59, 20, Math.toRadians(120));
    private final Pose goingToPick2 = new Pose(50, 70, Math.toRadians(180));

    private final Pose secondBatch = new Pose(14, 70, Math.toRadians(180));


    private PathChain goingToPick1, takeOne, goScoreOne, goingToPickSecond, takeSecond;

    public void buildPaths() {

        goingToPick1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, goingToPick))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        takeOne = follower.pathBuilder()
                .addPath(new BezierLine(goingToPick, firstBatch))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goScoreOne = follower.pathBuilder()
                .addPath(new BezierLine(firstBatch, shooting))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                .build();

        goingToPickSecond = follower.pathBuilder()
                .addPath(new BezierLine(shooting, goingToPick2))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .build();

        takeSecond= follower.pathBuilder()
                .addPath(new BezierLine(goingToPick2, secondBatch))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case -1: // settle
                if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                    setPathState(0);
                }
                break;

            case 0:
                    follower.followPath(goingToPick1);
                    setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(takeOne);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(goScoreOne);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(goingToPickSecond);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(takeSecond);
                    setPathState(5);
                }
                break;

            case 5:
                // Done
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
        follower.setMaxPower(0.75);
        buildPaths();
    }

    @Override
    public void start() {
        pathState = -1;
        pathTimer.resetTimer();
    }
}
