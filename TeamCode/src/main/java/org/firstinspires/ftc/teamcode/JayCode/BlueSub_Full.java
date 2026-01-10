package org.firstinspires.ftc.teamcode.JayCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.JayCode.Subsystems.IntakeSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.HoodSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.FlywheelSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.TurretSubsys;
import org.firstinspires.ftc.teamcode.JayCode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueSub_Full")
public class BlueSub_Full extends CommandOpMode {

    private Follower follower;
    private IntakeSubsys intake;
    private HoodSubsys hood;
    private FlywheelSubsys flywheel;
    private TurretSubsys turret;
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

    private void buildPaths() {
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

        RobotConstants.robotTeam = RobotConstants.Team.BLUE;

        telemetryData = new TelemetryData(telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.75);

        intake = new IntakeSubsys(hardwareMap);
        hood = new HoodSubsys(hardwareMap);
        flywheel = new FlywheelSubsys(hardwareMap);
        turret = new TurretSubsys(hardwareMap);

        buildPaths();

        double autoShotDistance = 120.0;

        RunCommand turretTracking = new RunCommand(() -> {
            Pose pose = follower.getPose();
            Pose2D robotPose = new Pose2D(
                    DistanceUnit.INCH,
                    pose.getX(),
                    pose.getY(),
                    AngleUnit.DEGREES,
                    Math.toDegrees(pose.getHeading())
            );
            turret.turretTrack(robotPose);
        }, turret);

        SequentialCommandGroup autoSequence = new SequentialCommandGroup(

                new InstantCommand(() -> intake.runIntake(1.0), intake),
                new FollowPathCommand(follower, alignWith1).setGlobalMaxPower(0.75),
                new FollowPathCommand(follower, firstSet),

                new InstantCommand(() -> {
                    hood.runHoodRegression(autoShotDistance);
                    flywheel.runFlywheelRegression(autoShotDistance);
                }, hood, flywheel),

                new FollowPathCommand(follower, secondSet),

                new InstantCommand(() -> {
                    intake.runIntake(0);
                    flywheel.setFlywheelVel(0);
                }, intake, flywheel)
        );

        schedule(new ParallelCommandGroup(
                turretTracking,
                autoSequence
        ));
    }

    @Override
    public void run() {
        super.run();

        follower.update();

        Pose pose = follower.getPose();
        telemetryData.addData("X", pose.getX());
        telemetryData.addData("Y", pose.getY());
        telemetryData.addData("Heading", Math.toDegrees(pose.getHeading()));
        telemetryData.addData("Flywheel Vel", flywheel.getFlywheelVel());
        telemetryData.addData("Turret Pos", turret.getTurretMotor().getCurrentPosition());
        telemetryData.update();
    }
}
