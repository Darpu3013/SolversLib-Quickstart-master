package org.firstinspires.ftc.teamcode.Code.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "StraightLineAuto")
public class AutoTest extends OpMode {

    private int pathState;
    private Follower follower;
    private Timer pathTimer;

    // Starting & ending poses
    private final Pose startPose = new Pose(38.5, 33, Math.toRadians(90));
    private final Pose goingPose  = new Pose(38.5, 70, Math.toRadians(90));

    private final Pose endPose = new Pose(60.5, 33, Math.toRadians(180));

    private PathChain straightLine, turn;

    public void buildPaths() {

        straightLine = follower.pathBuilder()
                .addPath(new BezierLine(startPose, goingPose))
                .setConstantHeadingInterpolation(Math.toRadians((90)))
                .build();

        turn = follower.pathBuilder()
                .addPath((new BezierCurve(
                        (goingPose),
                        new Pose(63, 82),
                        (endPose))))
                .setTangentHeadingInterpolation()
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
                    follower.followPath(turn);
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
