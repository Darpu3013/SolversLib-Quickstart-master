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

//@Autonomous(name = "BlueSubMeasure_THIS ")
public class BlueSubMeasure_THIS extends OpMode {

    private int pathState;
    private Follower follower;
    private Timer pathTimer;

    private DcMotor intakeMotor;
    private FlyWheel flywheel;

    private final Pose startPose = new Pose(27, 131, Math.toRadians(325));
    private final Pose align = new Pose(72, 105, Math.toRadians(180));
    private final Pose pickUpFirst = new Pose(35, 105, Math.toRadians(180));
    private final Pose align2 = new Pose(65, 85, Math.toRadians(180));
    private final Pose pickUpSecond = new Pose(35, 80, Math.toRadians(180));
    private final Pose backUpSecond = new Pose(70, 107, Math.toRadians(180));
    private final Pose align3 = new Pose(65, 63, Math.toRadians(180));
    private final Pose pickUpThird = new Pose(35, 63, Math.toRadians(180));
    private final Pose park = new Pose(62.5, 58.5, Math.toRadians(180));

    private PathChain alignWith1, firstSet, secondSet, thirdSet;

    public void buildPaths() {

        alignWith1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, align))
                .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(187))
                .build();

        firstSet = follower.pathBuilder()
                .addPath(new BezierLine(align, pickUpFirst))
                .setConstantHeadingInterpolation(Math.toRadians(190))
                .addPath(new BezierLine(pickUpFirst, align))
                .setConstantHeadingInterpolation(Math.toRadians(190))
                .build();

        secondSet = follower.pathBuilder()
                .addPath(new BezierLine(align, align2))
                .setConstantHeadingInterpolation(Math.toRadians(190))
                .addPath(new BezierLine(align2, pickUpSecond))
                .setConstantHeadingInterpolation(Math.toRadians(190))
                .addPath(new BezierLine(pickUpSecond, backUpSecond))
                .setConstantHeadingInterpolation(Math.toRadians(190))
                .build();

        thirdSet = follower.pathBuilder()
                .addPath(new BezierLine(backUpSecond, align3))
                .setConstantHeadingInterpolation(Math.toRadians(190))
                .addPath(new BezierLine(align3, pickUpThird))
                .setConstantHeadingInterpolation(Math.toRadians(190))
                .addPath(new BezierLine(pickUpThird, park))
                .setConstantHeadingInterpolation(Math.toRadians(190))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case -1:
                if (pathTimer.getElapsedTimeSeconds() > 0.15)
                    setPathState(0);
                break;

            case 0:
                follower.setMaxPower(0.75);
                follower.followPath(alignWith1);
                intakeMotor.setPower(-1.0);
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
        telemetry.update();
    }
}
