package org.firstinspires.ftc.teamcode.Code.Real;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@Autonomous(name = "BlueSmallTriangle_SL_NT")
public class BlueSmallTriangle_SL_NT extends CommandOpMode {

    private Follower follower;
    private DcMotor intakeMotor;
    private FlyWheel flywheel;
    private TelemetryData telemetryData;
    private final Pose startPose = new Pose(64, 9, Math.toRadians(90));
    private final Pose goingToPick = new Pose(40, 48, Math.toRadians(180));
    private final Pose firstBatch = new Pose(0, 48, Math.toRadians(180));
    private final Pose shooting = new Pose(56, 20, Math.toRadians(120));
    private final Pose goingToPick2 = new Pose(50, 70, Math.toRadians(180));
    private final Pose secondBatch = new Pose(0, 70, Math.toRadians(180));
    private final Pose alignWithLever = new Pose(18, 83, Math.toRadians(180));
    private final Pose pushLever = new Pose(8, 83, Math.toRadians(180));
    private final Pose lastShoot = new Pose(57, 13, Math.toRadians(90));
    private final Pose park = new Pose(40, 33.25, Math.toRadians(90));

    private PathChain goingToPick1, takeOne, goScoreOne;
    private PathChain goingToPickSecond, takeSecond, lever;
    private PathChain goToLastShoot, goToPark;

    public void buildPaths() {

        goingToPick1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, goingToPick))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        goingToPick.getHeading()
                )
                .build();

        takeOne = follower.pathBuilder()
                .addPath(new BezierLine(goingToPick, firstBatch))
                .setConstantHeadingInterpolation(firstBatch.getHeading())
                .build();

        goScoreOne = follower.pathBuilder()
                .addPath(new BezierLine(firstBatch, shooting))
                .setLinearHeadingInterpolation(
                        firstBatch.getHeading(),
                        shooting.getHeading()
                )
                .build();

        goingToPickSecond = follower.pathBuilder()
                .addPath(new BezierLine(shooting, goingToPick2))
                .setLinearHeadingInterpolation(
                        shooting.getHeading(),
                        goingToPick2.getHeading()
                )
                .build();

        takeSecond = follower.pathBuilder()
                .addPath(new BezierLine(goingToPick2, secondBatch))
                .setConstantHeadingInterpolation(secondBatch.getHeading())
                .build();

        lever = follower.pathBuilder()
                .addPath(new BezierLine(secondBatch, alignWithLever))
                .setConstantHeadingInterpolation(alignWithLever.getHeading())
                .addPath(new BezierLine(alignWithLever, pushLever))
                .setConstantHeadingInterpolation(pushLever.getHeading())
                .build();

        goToLastShoot = follower.pathBuilder()
                .addPath(new BezierLine(pushLever, lastShoot))
                .setLinearHeadingInterpolation(
                        pushLever.getHeading(),
                        lastShoot.getHeading()
                )
                .build();

        goToPark = follower.pathBuilder()
                .addPath(new BezierLine(lastShoot, park))
                .setConstantHeadingInterpolation(park.getHeading())
                .build();
    }

    @Override
    public void initialize() {
        super.reset();

        telemetryData = new TelemetryData(telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        flywheel = new FlyWheel(hardwareMap, "flywheel");

        buildPaths();

        schedule(new SequentialCommandGroup(

                new FollowPathCommand(follower, goingToPick1).setGlobalMaxPower(0.75),

                new FollowPathCommand(follower, takeOne, true, 0.75),

                new FollowPathCommand(follower, goScoreOne, true, 1.0),

                new FollowPathCommand(follower, goingToPickSecond),

                new FollowPathCommand(follower, takeSecond),

                new FollowPathCommand(follower, lever),

                new FollowPathCommand(follower, goToLastShoot),

                new FollowPathCommand(follower, goToPark)
        ));
    }

    @Override
    public void run() {
        super.run();

        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.addData("Intake Encoder", intakeMotor.getCurrentPosition());
        telemetryData.update();
    }
}
