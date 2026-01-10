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

//@Autonomous(name = "RedSmallTriangle_SL_NT")
public class RedSmallTriangle_SL_NT extends CommandOpMode {

    private Follower follower;
    private DcMotor intakeMotor;
    private FlyWheel flywheel;
    private TelemetryData telemetryData;

    private boolean isBlue = false;

    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));
    private final Pose goingToPickRaw = new Pose(55, 23, Math.toRadians(180));
    private final Pose firstBatchRaw = new Pose(25, 23, Math.toRadians(180));
    private final Pose shootingRaw = new Pose(55, 10, Math.toRadians(120));
    private final Pose goingToPick2Raw = new Pose(50, 40, Math.toRadians(180));
    private final Pose secondBatchRaw = new Pose(25, 45, Math.toRadians(180));
    private final Pose alignWithLeverRaw = new Pose(40, 55, Math.toRadians(180));
    private final Pose pushLeverRaw = new Pose(30, 55, Math.toRadians(180));
    private final Pose lastShootRaw = new Pose(55, 15, Math.toRadians(90));
    private final Pose alignParkRaw = new Pose(38, 15, Math.toRadians(90));
    private final Pose parkRaw = new Pose(39, 30.5, Math.toRadians(90));

    private Pose goingToPick, firstBatch, shooting;
    private Pose goingToPick2, secondBatch, alignWithLever;
    private Pose pushLever, lastShoot, alignPark, park;

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
                .addPath(new BezierLine(lastShoot, alignPark))
                .setConstantHeadingInterpolation(alignPark.getHeading())
                .addPath(new BezierLine(alignPark, park))
                .setConstantHeadingInterpolation(park.getHeading())
                .build();
    }

    @Override
    public void initialize() {
        super.reset();

        telemetryData = new TelemetryData(telemetry);

        follower = Constants.createFollower(hardwareMap);

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

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        flywheel = new FlyWheel(hardwareMap, "flywheel");

        buildPaths();

        schedule(new SequentialCommandGroup(

                new FollowPathCommand(follower, goingToPick1),

                new FollowPathCommand(follower, takeOne),

                new FollowPathCommand(follower, goScoreOne),

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

    private Pose maybeMirror(Pose pose) {
        return isBlue ? pose : pose.mirror();
    }
}
