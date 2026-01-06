package org.firstinspires.ftc.teamcode.Code.Real;

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

@Autonomous(name = "RedAutoEthan_NT")

public class RedAuto_Ethan_NT extends OpMode {

    private int pathState;
    private Follower follower;
    private Timer pathTimer;

    private DcMotor intakeMotor;
    private FlyWheel flywheel;

    private final Pose startPose = new Pose(80, 9, Math.toRadians(90));
    private final Pose goingToPick = new Pose(109, 35, Math.toRadians(0));
    private final Pose firstBatch = new Pose(126, 35, Math.toRadians(0));
    private final Pose shooting = new Pose(80, 13, Math.toRadians(60));
    private final Pose goingToPick2 = new Pose(102, 60, Math.toRadians(0));
    private final Pose secondBatch = new Pose(126, 60, Math.toRadians(0));
    private final Pose alignWithLever = new Pose(126, 68, Math.toRadians(0));
    private final Pose pushLever = new Pose(136, 68, Math.toRadians(0));
    private final Pose lastShoot = new Pose(80, 12, Math.toRadians(60));
    private final Pose park = new Pose(130, 12, Math.toRadians(90));



    private PathChain goingToPick1, takeOne, goScoreOne, goingToPickSecond, takeSecond, lever, goToLastShoot, goToPark;
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
                .addPath(new BezierLine(lastShoot, park))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(90))
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
                    follower.setMaxPower(0.75);
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

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setPower(0);

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
        telemetry.addData("Intake Encoder", intakeMotor.getCurrentPosition());
        telemetry.update();
    }
}

