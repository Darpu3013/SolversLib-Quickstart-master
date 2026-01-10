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

@Autonomous(name = "BlueSubMeasure_SL_NT")
public class BlueGoal_SL_NT extends CommandOpMode {

    private Follower follower;
    private DcMotor intakeMotor;
    private FlyWheel flywheel;
    private TelemetryData telemetryData;

    private final Pose startPose = new Pose(33, 144, Math.toRadians(325));
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
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        align.getHeading()
                )
                .build();

        firstSet = follower.pathBuilder()
                .addPath(new BezierLine(align, pickUpFirst))
                .setConstantHeadingInterpolation(pickUpFirst.getHeading())
                .addPath(new BezierLine(pickUpFirst, align))
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

        thirdSet = follower.pathBuilder()
                .addPath(new BezierLine(backUpSecond, align3))
                .setConstantHeadingInterpolation(align3.getHeading())
                .addPath(new BezierLine(align3, pickUpThird))
                .setConstantHeadingInterpolation(pickUpThird.getHeading())
                .addPath(new BezierLine(pickUpThird, park))
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

                new FollowPathCommand(follower, alignWith1).setGlobalMaxPower(0.75),

                new FollowPathCommand(follower, firstSet).setGlobalMaxPower(0.75),

                new FollowPathCommand(follower, secondSet).setGlobalMaxPower(0.75),

                new FollowPathCommand(follower, thirdSet).setGlobalMaxPower(0.75)
        ));
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
