package org.firstinspires.ftc.teamcode.Code.Real;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Big Triangle")
public class RedSubmersible extends OpMode{

    private int pathState;
    private Follower follower;
    private Timer pathTimer;

    private final Pose startPose = new Pose(117, 131, Math.toRadians(217));

    private final Pose align = new Pose(85, 84, Math.toRadians(0));

    private final Pose pickUpFirst = new Pose(112, 84, Math.toRadians(0));
    private final Pose backUpFirst = new Pose(85, 84, Math.toRadians(0));

    private final Pose align2 = new Pose(42, 59, Math.toRadians(180));

    private final Pose pickUpSecond = new Pose(31, 59, Math.toRadians(180));

    private final Pose backUpSecond = new Pose(70, 77, Math.toRadians(180));

    private final Pose  align3 = new Pose(42, 35, Math.toRadians(180));

    private final Pose pickUpThird = new Pose(12, 35, Math.toRadians(180));

    private final Pose park = new Pose(12, 10, Math.toRadians(180));



    private PathChain alignWith1, firstSet, secondSet, thirdSet;

    public void buildPaths() {

        alignWith1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, align))
                .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(180))
                .build();

        firstSet = follower.pathBuilder()
                .addPath(new BezierLine(align, pickUpFirst))
                .addPath(new BezierLine(pickUpFirst, backUpFirst ))
                .setConstantHeadingInterpolation(180)
                .build();

        secondSet = follower.pathBuilder()
                .addPath(new BezierLine(backUpFirst, align2))
                .addPath(new BezierLine(align2, pickUpSecond))
                .addPath(new BezierLine(pickUpSecond, backUpSecond))
                .setConstantHeadingInterpolation(180)
                .build();

        thirdSet = follower.pathBuilder()
                .addPath(new BezierLine(align2, align3))
                .addPath(new BezierLine(align3, pickUpThird))
                .addPath(new BezierLine(pickUpThird, park))
                .setConstantHeadingInterpolation(180)
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
                follower.followPath(alignWith1);
                setPathState(1);
                break;

            case 1:
                follower.followPath(firstSet);
                setPathState(1);
                break;

            case 2:
                follower.followPath(secondSet);
                setPathState(1);
                break;

            case 3:
                follower.followPath(thirdSet);
                setPathState(1);
                break;

            case 4:
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
