package org.firstinspires.ftc.teamcode.Code.Turret;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Turret Goal Lock")
public class Goal extends OpMode {

    // ---------------- CONSTANTS ----------------
    public static final double GOAL_X = 12;
    public static final double GOAL_Y = 136;

    // ---------------- HARDWARE ----------------
    private Turret turret;        // your turret subsystem
    private Follower follower;    // PedroPathing odometry

    @Override
    public void init() {
        // Initialize turret subsystem
        turret = new Turret(hardwareMap, "turretMotor");

        // Initialize PedroPathing follower (odometry)
        follower = Constants.createFollower(hardwareMap);

        // Set starting pose — make sure the heading is in radians
        follower.setStartingPose(new Pose(64, 9, Math.toRadians(90)));

        telemetry.addLine("Init complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update follower (odometry)
        follower.update();

        // Get current robot pose
        Pose robotPose = follower.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading(); // radians

        // Calculate angle from robot to goal (field-relative)
        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;
        double fieldAngle = Math.atan2(dy, dx);

        // Convert to robot-relative turret angle
        double turretAngleRad = fieldAngle - robotHeading;

        // Normalize to [-π, π] so it chooses shortest rotation
        while (turretAngleRad > Math.PI) turretAngleRad -= 2 * Math.PI;
        while (turretAngleRad < -Math.PI) turretAngleRad += 2 * Math.PI;

        // Convert to degrees for Turret subsystem
        double turretAngleDeg = Math.toDegrees(turretAngleRad);

        // Command the turret to move
        turret.setTurretAngle(turretAngleDeg);

        // Telemetry
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotHeading));
        telemetry.addData("Turret Angle (deg)", turretAngleDeg);
        telemetry.update();
    }
}
