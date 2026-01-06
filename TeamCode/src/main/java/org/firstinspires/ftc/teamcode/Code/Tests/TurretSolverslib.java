package org.firstinspires.ftc.teamcode.Code.Tests;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Subsystems.TurretNew;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="TurretSolverslib")
public class TurretSolverslib extends LinearOpMode {

    private TurretNew turret;
    private Follower follower;

    private final Pose startPose = new Pose(64, 9, Math.toRadians(90));

    @Override
    public void runOpMode() {

        turret = new TurretNew(hardwareMap, "turretMotor");
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        double goalX = 0;
        double goalY = 144;

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeading = follower.getPose().getHeading();

            double dx = goalX - robotX;
            double dy = goalY - robotY;

            double angleToGoal = Math.atan2(dy, dx);

            double turretAngle = angleToGoal - robotHeading;

            turretAngle = Math.atan2(Math.sin(turretAngle), Math.cos(turretAngle));

            double targetDeg = Math.toDegrees(turretAngle);

            turret.setTurretAngle(targetDeg);

            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Robot Heading", Math.toDegrees(robotHeading));
            telemetry.addData("Target Angle", targetDeg);
            telemetry.addData("Current Angle", turret.getTurretAngle());
            telemetry.update();
        }

        turret.stop();
    }
}
