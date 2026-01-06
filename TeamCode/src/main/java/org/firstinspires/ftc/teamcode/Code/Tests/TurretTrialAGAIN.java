package org.firstinspires.ftc.teamcode.Code.Tests;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TurretKeepTryingNM")
public class TurretTrialAGAIN extends LinearOpMode {

    private Turret turret;
    private Follower follower;

    private final Pose startPose = new Pose(64, 9, Math.toRadians(90));

    @Override
    public void runOpMode() {

        turret = new Turret(hardwareMap, "turretMotor");
        follower = Constants.createFollower(hardwareMap);

        follower.setPose(startPose);

        // target
        double xTarget = 0;
        double yTarget = 144;

        // turret constants (from your subsystem)
        double TICKS_PER_REV_Turret = 141.1 * 5.2;
        double turretMaxTicks = (150.0 / 360.0) * TICKS_PER_REV_Turret;
        double offsetConstant = 0;

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            // ===============================
            // ROBOT STATE
            // ===============================
            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeading = follower.getPose().getHeading();

            double robotXVelo = 0;
            double robotYVelo = 0;

            // ===============================
            // YOUR EXACT LOGIC (UNCHANGED)
            // ===============================
            double dy = yTarget - robotY - robotYVelo;
            double dx = xTarget - robotX - robotXVelo;
            double horizontalDistance = Math.hypot(dy, dx);

            double fieldAngle = Math.atan2(dy, dx);
            double robotdAngle = (fieldAngle - robotHeading);

            double turretCurrAngle = fieldAngle - robotdAngle - Math.PI;

            if (turretCurrAngle < Math.toRadians(-180)) {
                turretCurrAngle += 2 * Math.PI;
            }

            if (turretCurrAngle > Math.toRadians(180)) {
                turretCurrAngle -= 2 * Math.PI;
            }

            double dtheta1 = (turretCurrAngle - fieldAngle) % (2 * Math.PI);
            double dtheta2 = ((turretCurrAngle + 2 * Math.PI) - fieldAngle) % (2 * Math.PI);
            double dtheta = Math.abs(dtheta1) > Math.abs(dtheta2) ? dtheta2 : dtheta1;

            double motorRevs = (dtheta / (2.0 * Math.PI)) * 3.0;

            double targetTicks = (int) (motorRevs * TICKS_PER_REV_Turret);
            targetTicks = Math.max(
                    -turretMaxTicks + offsetConstant,
                    Math.min(turretMaxTicks + offsetConstant, targetTicks)
            );

            // ===============================
            // APPLY TO TURRET
            // ===============================
            double targetAngleDeg =
                    targetTicks * (360.0 / TICKS_PER_REV_Turret);

            turret.setTurretAngle(targetAngleDeg);

            // ===============================
            // TELEMETRY
            // ===============================
            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Robot Heading", Math.toDegrees(robotHeading));
            telemetry.addData("Turret Angle", turret.getTurretAngle());
            telemetry.addData("Target Angle", targetAngleDeg);
            telemetry.update();
        }
    }
}
