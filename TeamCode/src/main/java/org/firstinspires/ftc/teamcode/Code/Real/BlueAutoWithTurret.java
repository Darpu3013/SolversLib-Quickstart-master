package org.firstinspires.ftc.teamcode.Code.Real;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueSub_Intake")
public class BlueAutoWithTurret extends CommandOpMode {

    private Follower follower;
    private Intake intake;
    private FlyWheel flywheel;
    private TelemetryData telemetryData;

    private final Pose startPose = new Pose(33, 144, Math.toRadians(270));
    private final Pose align = new Pose(72, 108, Math.toRadians(180));
    private final Pose pickUpFirst = new Pose(40, 108, Math.toRadians(180));
    private final Pose hitEmpty = new Pose(35, 103, Math.toRadians(180));
    private final Pose align2 = new Pose(65, 85, Math.toRadians(180));
    private final Pose pickUpSecond = new Pose(35, 85, Math.toRadians(180));
    private final Pose backUpSecond = new Pose(70, 107, Math.toRadians(180));

    private PathChain alignWith1;
    private PathChain firstSet;
    private PathChain secondSet;

    public void buildPaths() {

        alignWith1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, align))
                .setLinearHeadingInterpolation(startPose.getHeading(), align.getHeading())
                .build();

        firstSet = follower.pathBuilder()
                .addPath(new BezierLine(align, pickUpFirst))
                .setConstantHeadingInterpolation(pickUpFirst.getHeading())
                .addPath(new BezierLine(pickUpFirst, hitEmpty))
                .setConstantHeadingInterpolation(hitEmpty.getHeading())
                .addPath(new BezierLine(hitEmpty, align))
                .setConstantHeadingInterpolation(align.getHeading())
                .build();

        secondSet = follower.pathBuilder()
                .addPath(new BezierLine(align, align2))
                .setConstantHeadingInterpolation(align2.getHeading())
                .addPath(new BezierLine(align2, pickUpSecond))
                .setConstantHeadingInterpolation(pickUpSecond.getHeading())
                .addPath(new BezierLine(pickUpSecond, backUpSecond))
                .setConstantHeadingInterpolation(backUpSecond.getHeading())
                .build();
    }

    @Override
    public void initialize() {
        super.reset();

        telemetryData = new TelemetryData(telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.75);

        intake = new Intake(hardwareMap, "intakeMotor");
        flywheel = new FlyWheel(hardwareMap, "flywheel");

        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new InstantCommand(() -> intake.intake()),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, alignWith1).setGlobalMaxPower(0.75),
                                new FollowPathCommand(follower, firstSet),
                                new FollowPathCommand(follower, secondSet)
                        )
                ),
                new InstantCommand(() -> intake.stop())
        );

        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();

        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}
