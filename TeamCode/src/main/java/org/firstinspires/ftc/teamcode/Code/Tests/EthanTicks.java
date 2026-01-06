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
import org.firstinspires.ftc.teamcode.Subsystems.IntakeUpdated;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "EthanTicks")

public class EthanTicks extends OpMode {

    private int pathState;
    private Follower follower;
    private Timer pathTimer;

    private IntakeUpdated intake;

    private FlyWheel flywheel;

    private final Pose startPose = new Pose(64, 9, Math.toRadians(90));
    private final Pose goingToPick = new Pose(40, 48, Math.toRadians(180));
    private final Pose firstBatch = new Pose(-5, 48, Math.toRadians(180));
    private final Pose shooting = new Pose(59, 20, Math.toRadians(120));
    private final Pose goingToPick2 = new Pose(50, 70, Math.toRadians(180));
    private final Pose secondBatch = new Pose(0, 70, Math.toRadians(180));
    private final Pose alignWithLever = new Pose(14, 85, Math.toRadians(180));

    private final Pose pushLever = new Pose(7, 85, Math.toRadians(180));

    private final Pose lastShoot = new Pose(59, 25, Math.toRadians(180));

    private final Pose park = new Pose(3, 25, Math.toRadians(180));



    private PathChain goingToPick1, takeOne, goScoreOne, goingToPickSecond, takeSecond, lever, goToLastShoot, goToPark;
    private void buildPaths() {
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

        takeSecond = follower.pathBuilder()
                .addPath(new BezierLine(goingToPick2, secondBatch))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        lever = follower.pathBuilder()
                .addPath(new BezierLine(secondBatch, alignWithLever))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(alignWithLever, pushLever))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToLastShoot = follower.pathBuilder()
                .addPath(new BezierLine(pushLever, lastShoot))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToPark = follower.pathBuilder()
                .addPath(new BezierLine(lastShoot, park))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                if (pathTimer.getElapsedTimeSeconds() > 0.15) setPathState(0);
                break;
            case 0:
                follower.followPath(goingToPick1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.45);
                    follower.followPath(takeOne);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(goScoreOne);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    flywheel.setPower(1.0);
                    intake.startFeeding();
                    setPathState(4);
                }
                break;
            case 4:
                if (intake.isFeeding()) {
                    intake.update();
                } else {
                    flywheel.stop();
                    follower.followPath(goingToPickSecond);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(takeSecond);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(lever);
                    setPathState(7);
                }
                break;


            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(goToLastShoot);
                    setPathState(8);
                }
                break;


            case 8:
                if (!follower.isBusy()) {
                    flywheel.setPower(1.0);
                    intake.startFeeding();
                    setPathState(9);
                }
                break;

            case 9:
                if (intake.isFeeding()) {
                    intake.update();
                } else {
                    flywheel.stop();
                    follower.followPath(goToPark);
                    setPathState(10);
                }
                break;


            case 10:
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.75);

        intake = new IntakeUpdated(hardwareMap, "intakeMotor");


        flywheel = new FlyWheel(hardwareMap, "flywheel");

        buildPaths();
    }

    @Override
    public void start() {
        pathState = -1;
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
        telemetry.addData("Feeding?", intake.isFeeding());
        telemetry.update();
    }
}

