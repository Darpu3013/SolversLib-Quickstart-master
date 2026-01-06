package org.firstinspires.ftc.teamcode.Code.Real;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto_Jay")
public class Auto_Jay extends OpMode {

    private int pathState;
    private Follower follower;
    private Timer pathTimer;

    private final Pose startPose = new Pose(72, 9, Math.toRadians(90));
    private final Pose goingToPick = new Pose(39, 35, Math.toRadians(180));
    private final Pose firstPickCurve = new Pose(53, 4, Math.toRadians(130));
    private final Pose square = new Pose(12, 20, Math.toRadians(180));

    private final Pose leadIn = new Pose(56, 13, Math.toRadians(90));

    private final Pose score = new Pose(63, 25, Math.toRadians(130));

    private final Pose forLil = new Pose(72, 30, Math.toRadians(90));


    private PathChain goingToPick1, takeAndLaunch1, takeAndPark, squareToSmallTriangle, park;

    public void buildPaths() {

        goingToPick1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, forLil))
                .addPath(new BezierLine(forLil, goingToPick))
                .setTangentHeadingInterpolation()
                .build();

        takeAndLaunch1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        goingToPick,
                        new Pose(12, 30),
                        score))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();

        takeAndPark = follower.pathBuilder()
                .addPath(new BezierCurve(
                        score,
                        new Pose(2, 38),
                        new Pose(13, 10),
                        square))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .build();

        squareToSmallTriangle = follower.pathBuilder()
                .addPath(new BezierLine(square, score))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(score, new Pose(2, 19)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
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
                    follower.followPath(takeAndLaunch1);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(takeAndPark);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(squareToSmallTriangle);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(park);
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
