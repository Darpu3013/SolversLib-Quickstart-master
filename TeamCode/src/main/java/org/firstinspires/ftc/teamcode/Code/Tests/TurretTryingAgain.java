package org.firstinspires.ftc.teamcode.Code.Tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="TurretTryingAgain")
public class TurretTryingAgain extends LinearOpMode {

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

        double goalX = 64;
        double goalY = 9;

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeading = follower.getPose().getHeading();

            // Compute angle to goal relative to robot
            double deltaX = goalX - robotX;
            double deltaY = goalY - robotY;
            double angleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));
            double turretTarget = wrapDegrees(angleToGoal - Math.toDegrees(robotHeading));

            double currentAngle = turret.getTurretAngle();

            // Compute shortest-path error
            double error = wrapDegrees(turretTarget - currentAngle);

            // PID calculations
            integral += error;
            double derivative = error - lastError;
            lastError = error;

            double pidOutput = kP * error + kI * integral + kD * derivative;
            pidOutput = Math.max(-1, Math.min(1, pidOutput));

            turret.setMotorPower(pidOutput);

            // Telemetry
            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Robot Heading", Math.toDegrees(robotHeading));
            telemetry.addData("Target Angle", turretTarget);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error", error);
            telemetry.addData("PID Output", pidOutput);
            telemetry.update();
        }
    }

    // Wrap angle to [-180, 180] degrees
    private double wrapDegrees(double angle) {
        angle = ((angle + 180) % 360 + 360) % 360 - 180;
        return angle;
    }
}
