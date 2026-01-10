package org.firstinspires.ftc.teamcode.Code.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@Autonomous(name = "RedSubMeasure_THIS")
public class RedSubMeasure_THIS extends OpMode {

    private boolean isBlue = false;

    private int pathState;
    private Follower follower;
    private Timer pathTimer;

    private DcMotor intakeMotor;
    private FlyWheel flywheel;

    private final Pose startPose = new Pose(117, 131, Math.toRadians(217));
    private final Pose alignRaw = new Pose(57, 63, Math.toRadians(0));
    private final Pose pickUpFirstRaw = new Pose(23, 63, Math.toRadians(0));
    private final Pose align2Raw = new Pose(57, 40, Math.toRadians(0));
    private final Pose pickUpSecondRaw = new Pose(23, 40, Math.toRadians(0));
    private final Pose backUpSecondRaw = new Pose(60, 75, Math.toRadians(0));
    private final Pose align3Raw = new Pose(57, 15, Math.toRadians(0));
    private final Pose pickUpThirdRaw = new Pose(23, 15.5, Math.toRadians(0));
    private final Pose parkRaw = new Pose(44, 15.5, Math.toRadians(0));
    

    private Pose align, pickUpFirst, align2, pickUpSecond;
    private Pose backUpSecond, align3, pickUpThird, park;

    private PathChain alignWith1, firstSet, secondSet, thirdSet;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setPower(0);

        flywheel = new FlyWheel(hardwareMap, "flywheel");

        align = maybeMirror(alignRaw);
        pickUpFirst = maybeMirror(pickUpFirstRaw);
        align2 = maybeMirror(align2Raw);
        pickUpSecond = maybeMirror(pickUpSecondRaw);
        backUpSecond = maybeMirror(backUpSecondRaw);
        align3 = maybeMirror(align3Raw);
        pickUpThird = maybeMirror(pickUpThirdRaw);
        park = maybeMirror(parkRaw);

        follower.setStartingPose(startPose);
        follower.setMaxPower(0.75);

        buildPaths();
    }

    private void buildPaths() {

        alignWith1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, align))
                .setLinearHeadingInterpolation(Math.toRadians(217), Math.toRadians(0))
                .build();

        firstSet = follower.pathBuilder()
                .addPath(new BezierLine(align, pickUpFirst))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(pickUpFirst, align))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        secondSet = follower.pathBuilder()
                .addPath(new BezierLine(align, align2))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(align2, pickUpSecond))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(pickUpSecond, backUpSecond))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        thirdSet = follower.pathBuilder()
                .addPath(new BezierLine(backUpSecond, align3))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(align3, pickUpThird))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(pickUpThird, park))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case -1:
                if (pathTimer.getElapsedTimeSeconds() > 0.15)
                    setPathState(0);
                break;

            case 0:
                follower.setMaxPower(0.75);
                follower.followPath(alignWith1);
                intakeMotor.setPower(-1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(firstSet);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    flywheel.setPower(1.0);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    flywheel.stop();
                    follower.setMaxPower(0.75);
                    follower.followPath(secondSet);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    flywheel.setPower(1.0);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    flywheel.stop();
                    follower.setMaxPower(0.75);
                    follower.followPath(thirdSet);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    flywheel.setPower(1.0);
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    flywheel.stop();
                    setPathState(8);
                }
                break;

            case 8:
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("H", follower.getPose().getHeading());
        telemetry.update();
    }

    private Pose maybeMirror(Pose pose) {
        return isBlue ? pose : pose.mirror();
    }
}
