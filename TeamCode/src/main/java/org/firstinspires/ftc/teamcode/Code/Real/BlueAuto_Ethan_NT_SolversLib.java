package org.firstinspires.ftc.teamcode.Code.Real;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

// SolversLib Imports
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueAutoEthan_SolversLib")
public class BlueAuto_Ethan_NT_SolversLib extends CommandOpMode {

    private Follower follower;
    private DcMotor intakeMotor;
    private FlyWheel flywheel;
    private TelemetryData telemetryData;

    private final Pose startPose = new Pose(64, 9, Math.toRadians(90));
    private final Pose goingToPick = new Pose(40, 48, Math.toRadians(180));
    private final Pose firstBatch = new Pose(-5, 48, Math.toRadians(180));
    private final Pose shooting = new Pose(59, 20, Math.toRadians(120));
    private final Pose goingToPick2 = new Pose(50, 70, Math.toRadians(180));
    private final Pose secondBatch = new Pose(0, 70, Math.toRadians(180));
    private final Pose alignWithLever = new Pose(14, 85, Math.toRadians(180));
    private final Pose pushLever = new Pose(7, 85, Math.toRadians(180));
    private final Pose lastShoot = new Pose(57, 13, Math.toRadians(90));
    private final Pose park = new Pose(13, 13, Math.toRadians(90));

    private PathChain goingToPick1, takeOne, goScoreOne, goingToPickSecond, takeSecond, lever, goToLastShoot, goToPark;

    public void buildPaths() {

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
                .addPoseCallback(goingToPick, () -> intakeMotor.setPower(-1), 0.0)
                .addPath(new BezierLine(alignWithLever, pushLever))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToLastShoot = follower.pathBuilder()
                .addPath(new BezierLine(pushLever, lastShoot))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        goToPark = follower.pathBuilder()
                .addPath(new BezierLine(lastShoot, park))
                .setConstantHeadingInterpolation(Math.toRadians(90))
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
                new WaitCommand(150),

                new FollowPathCommand(follower, goingToPick1).setGlobalMaxPower(0.75),

                new FollowPathCommand(follower, takeOne, true, 0.45),

                new FollowPathCommand(follower, goScoreOne, true, 1.0),

                new InstantCommand(() -> flywheel.setPower(1.0)),
                new WaitCommand(500),
                new InstantCommand(() -> flywheel.stop()),

                new FollowPathCommand(follower, goingToPickSecond),
                new FollowPathCommand(follower, takeSecond),
                new FollowPathCommand(follower, lever),
                new FollowPathCommand(follower, goToLastShoot),

                new InstantCommand(() -> flywheel.setPower(1.0)),
                new WaitCommand(500),
                new InstantCommand(() -> flywheel.stop()),

                new FollowPathCommand(follower, goToPark)
        ));
    }

    @Override
    public void run() {
        super.run();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.addData("Intake Encoder", intakeMotor.getCurrentPosition());
        telemetryData.update();
    }
}