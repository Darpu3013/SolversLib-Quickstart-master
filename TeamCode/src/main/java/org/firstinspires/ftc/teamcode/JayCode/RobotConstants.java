package org.firstinspires.ftc.teamcode.JayCode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
public class RobotConstants {

    public enum Team {
        RED, BLUE
    }

    public static Team robotTeam = Team.RED;

    public static final String limelightName = "limelight";
    public static int redPipeline = 7;
    public static int bluePipeline = 6;

    public static final String pinpointName = "pinpoint";

    public static Pose3D limelightPose = new Pose3D(new Position(DistanceUnit.INCH, 0, 0, 0, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));

    public static final String hoodServoName = "hoodServo";
    public static final boolean hoodReversed = true;

    public static double flywheelPCoeff = 0.0000055, flywheelDCoeff = 0;
    public static double flywheelKV = 0.0004;
    public static final String leftFlywheelName = "leftFlywheel";
    public static final String rightFlyWheelName = "rightFlywheel";
    public static final boolean leftFlywheelInverted = true;

    public static Pose2D robotPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    public static final String turretName = "turretMotor";
    public static final Motor.GoBILDA turretMotType = Motor.GoBILDA.RPM_1150;
    public static boolean turretReversed = true;

    public static double turretTolerance = 3;
    public static double turretLeftMax = -90; //degrees
    public static double turretRightMax = 90;
    public static double turretPCoeff = 0.018, turretDCoeff = 0;
    public static final double turretRatio = 2/1;
    public static final double turretTicksPerDegree = (turretMotType.getCPR() * turretRatio) / 360;

    public static final double goalX = -72;
    public static final double blueGoalY = -72;
    public static final double redGoalY = 72;

    public static final String intakeName = "intakeMotor";
    public static boolean intakeReversed = true;

    public static final double hoodMin = 0.08;
    public static final double hoodMax = 0.754;

    public static double stopperTime = 0.5;
    public static final String stopperName = "kickerServo";
    public static final double stopperClose = 0.285;
    public static final double stopperOpen = 0.59;

    public static final String flName = "flMotor";
    public static final String blName = "blMotor";
    public static final String brName = "brMotor";
    public static final String frName = "frMotor";
    public static boolean leftDTReversed = true;
    public static final Motor.ZeroPowerBehavior teleopBrakeBehavior = Motor.ZeroPowerBehavior.BRAKE;
    public static boolean fieldCentric = false;
    public static double maxDriveSpeed = 1.0;
}
