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

//@Autonomous(name = "Mirror_Measure_THIS")
public class Mirror_measure_THIS extends OpMode {

    private boolean isBlue = false;

    private int pathState;
    private Follower follower;
    private Timer pathTimer;

    private DcMotor intakeMotor;
    private FlyWheel flywheel;

    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));
    private final Pose goingToPickRaw = new Pose(55, 23, Math.toRadians(180));
    private final Pose firstBatchRaw = new Pose(25, 23, Math.toRadians(180));
    private final Pose shootingRaw = new Pose(55, 10, Math.toRadians(120));
    private final Pose goingToPick2Raw = new Pose(50, 40, Math.toRadians(180));
    private final Pose secondBatchRaw = new Pose(25, 45, Math.toRadians(180));
    private final Pose alignWithLeverRaw = new Pose(40, 55, Math.toRadians(180));
    private final Pose pushLeverRaw = new Pose(27, 55, Math.toRadians(180));
    private final Pose lastShootRaw = new Pose(55, 15, Math.toRadians(90));
    private final Pose alignParkRaw = new Pose(38, 15, Math.toRadians(90));
    private final Pose parkRaw = new Pose(38, 32, Math.toRadians(90));

    private Pose goingToPick, firstBatch, shooting;
    private Pose goingToPick2, secondBatch, alignWithLever;
    private Pose pushLever, lastShoot, alignPark, park;

    private PathChain goingToPick1, takeOne, goScoreOne,
            goingToPickSecond, takeSecond, lever,
            goToLastShoot, goToPark;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setPower(0);

        flywheel = new FlyWheel(hardwareMap, "flywheel");

        goingToPick = maybeMirror(goingToPickRaw);
        firstBatch = maybeMirror(firstBatchRaw);
        shooting = maybeMirror(shootingRaw);
        goingToPick2 = maybeMirror(goingToPick2Raw);
        secondBatch = maybeMirror(secondBatchRaw);
        alignWithLever = maybeMirror(alignWithLeverRaw);
        pushLever = maybeMirror(pushLeverRaw);
        lastShoot = maybeMirror(lastShootRaw);
        alignPark = maybeMirror(alignParkRaw);
        park = maybeMirror(parkRaw);

        follower.setStartingPose(startPose);
        follower.setMaxPower(0.75);

        buildPaths();
    }

    private void buildPaths() {
        goingToPick1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, goingToPick))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        takeOne = follower.pathBuilder()
                .addPath(new BezierLine(goingToPick, firstBatch))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPoseCallback(goingToPick, () -> intakeMotor.setPower(-1), 0.0)
                .build();

        goScoreOne = follower.pathBuilder()
                .addPath(new BezierLine(firstBatch, shooting))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                .build();

        goingToPickSecond = follower.pathBuilder()
                .addPath(new BezierLine(shooting, goingToPick2))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                .build();

        takeSecond = follower.pathBuilder()
                .addPath(new BezierLine(goingToPick2, secondBatch))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        lever = follower.pathBuilder()
                .addPath(new BezierLine(secondBatch, alignWithLever))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPoseCallback(goingToPick, () -> intakeMotor.setPower(-1), 0.0)
                .addPath(new BezierLine(alignWithLever, pushLever))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goToLastShoot = follower.pathBuilder()
                .addPath(new BezierLine(pushLever, lastShoot))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                .build();

        goToPark = follower.pathBuilder()
                .addPath(new BezierLine(lastShoot, alignPark))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(new BezierLine(alignPark, park))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case -1:
                if (pathTimer.getElapsedTimeSeconds() > 0.15)
                    setPathState(0);
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
                    flywheel.setPower(1.0);
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
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
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    flywheel.stop();
                    follower.followPath(goToPark);
                    setPathState(10);
                }
                break;

            case 10:
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
