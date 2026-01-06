package org.firstinspires.ftc.teamcode.Code.Real;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
/*
@Autonomous(name = "BlueAuto_WithTurret")
public class BlueAuto_Ethan_Full extends OpMode {

    private int pathState;
    private Follower follower;
    private Timer pathTimer;
    private DcMotor intakeMotor;
    private FlyWheel flywheel;

    private DcMotor turretMotor;
    private Limelight3A limelight3A;
    private BNO055IMU imu;

    private double KpTurret = 0.01;
    private double minPower = 0.05;
    private double robotYawOffset;
    private boolean firstLoop = true;

    private final Pose startPose = new Pose(64, 9, Math.toRadians(90));
    private final Pose goingToPick = new Pose(40, 48, Math.toRadians(180));
    private final Pose firstBatch = new Pose(-5, 48, Math.toRadians(180));
    private final Pose shooting = new Pose(59, 20, Math.toRadians(120));
    private final Pose goingToPick2 = new Pose(50, 70, Math.toRadians(180));
    private final Pose secondBatch = new Pose(14, 70, Math.toRadians(180));
    private final Pose alignWithLever = new Pose(14, 85, Math.toRadians(180));

    private PathChain goingToPick1, takeOne, goScoreOne, goingToPickSecond, takeSecond, lever;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.75);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setPower(0);

        flywheel = new FlyWheel(hardwareMap, "flywheel");

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        robotYawOffset = imu.getAngularOrientation().firstAngle;

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
        limelight3A.start();

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
        updateTurret();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    private void updateTurret() {
        double robotYaw = imu.getAngularOrientation().firstAngle - robotYawOffset;
        LLResult llResult = limelight3A.getLatestResult();
        double tx = 0.0;
        if (llResult != null && llResult.isValid()) {
            tx = llResult.getTx();
        }

        double targetAngle = robotYaw + tx;
        double currentTurretAngle = turretMotor.getCurrentPosition();
        double error = targetAngle - currentTurretAngle;

        double turretPower = 0;
        if (!firstLoop && Math.abs(error) > 1) {
            turretPower = KpTurret * error;
            if (Math.abs(turretPower) < minPower) {
                turretPower = Math.signum(turretPower) * minPower;
            }
            turretPower = Math.max(-1.0, Math.min(1.0, turretPower));
        }

        turretMotor.setPower(turretPower);
        firstLoop = false;
    }

    private void buildPaths() {
        goingToPick1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, goingToPick))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        takeOne = follower.pathBuilder()
                .addPath(new BezierLine(goingToPick, firstBatch))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPoseCallback(goingToPick, () -> intakeMotor.setPower(-1), 0.0)
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
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    flywheel.stop();
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
                if (!follower.isBusy()) {
                    follower.followPath(lever);
                    setPathState(6);
                }
                break;
            case 6:
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}

 */