package org.firstinspires.ftc.teamcode.Code.Tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="TurretKeepTryingNew")
public class TurretKeepTryingNew extends LinearOpMode {

    private Turret turret;
    private Follower follower;
    private final Pose startPose = new Pose(64, 9, Math.toRadians(90));

    private double kP = 0.04;
    private double kI = 0.001;
    private double kD = 0.0;
    private double integral = 0;
    private double lastError = 0;

    @Override
    public void runOpMode() {
        turret = new Turret(hardwareMap, "turretMotor");
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        double goalX = 0;
        double goalY = 144;

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeading = follower.getPose().getHeading(); // radians

            double deltaX = goalX - robotX;
            double deltaY = goalY - robotY;

            double angleToGoal = Math.atan2(deltaY, deltaX);

            double turretAngleRelative = angleToGoal - robotHeading;

            turretAngleRelative = Math.atan2(Math.sin(turretAngleRelative), Math.cos(turretAngleRelative));

            double targetAngleDeg = Math.toDegrees(turretAngleRelative);

            double currentAngle = turret.getTurretAngle();
            double error = targetAngleDeg - currentAngle;

            error = Math.atan2(Math.sin(Math.toRadians(error)), Math.cos(Math.toRadians(error))) * (180 / Math.PI);

            // integral += error;
            double derivative = error - lastError;
            lastError = error;

            double pidOutput = kP * error + kI * integral + kD * derivative;

            pidOutput = Math.max(-1, Math.min(1, pidOutput));

            turret.setMotorPower(pidOutput);

            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Robot Heading", Math.toDegrees(robotHeading));
            telemetry.addData("Target Angle", targetAngleDeg);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error", error);
            telemetry.addData("PID Output", pidOutput);
            telemetry.update();
        }
    }
}