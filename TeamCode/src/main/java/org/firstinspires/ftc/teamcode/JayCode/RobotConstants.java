package org.firstinspires.ftc.teamcode.JayCode;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Configurable
public class RobotConstants {

    public static final String turretName = "turretMotor";
    public static final Motor.GoBILDA turretMotType = Motor.GoBILDA.RPM_1150;
    public static boolean turretReversed = true;

    public static double turretLeftMax = -100; //degrees
    public static double turretRightMax = 110;
    public static double turretP = 0.02, turretD = 0.001;
    public static final double turretRatio = 2/1;
    public static final double turretTicksPerDegree = (turretMotType.getCPR() * turretRatio) / 360;

    public static final double blueGoalX = -72;
    public static final double blueGoalY = -72;

    public static final double hoodMin = 0.08;
    public static final double hoodMax = 0.754;
}
