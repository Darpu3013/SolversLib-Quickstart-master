package org.firstinspires.ftc.teamcode.JayCode.Subsystems;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PDController;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.JayCode.RobotConstants;

public class TurretSubsys extends SubsystemBase {
    private Motor turretMotor;
    private PDController pdController;
    private double goalX = RobotConstants.goalX, goalY;

    public TurretSubsys(HardwareMap hwMap){
        this.turretMotor = new Motor(hwMap, RobotConstants.turretName, RobotConstants.turretMotType)
                .setInverted(RobotConstants.turretReversed);
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.stopAndResetEncoder();

        if (RobotConstants.robotTeam == RobotConstants.Team.RED) {goalY = RobotConstants.redGoalY;}
        else {goalY = RobotConstants.blueGoalY;}

        pdController = new PDController(RobotConstants.turretPCoeff, RobotConstants.turretDCoeff);
        pdController.setTolerance(RobotConstants.turretTolerance);
    }

    public void turretToAngle(double targetAngle){
        double clampedTarg = MathUtils.clamp(targetAngle, RobotConstants.turretLeftMax, RobotConstants.turretRightMax);
        int goal = (int)(clampedTarg * RobotConstants.turretTicksPerDegree);
        double current = turretMotor.getCurrentPosition();
        double output = pdController.calculate(current, goal);
        turretMotor.set(output);
    }

    public void turretTrack(Pose2D robotPose){
        double robotX = robotPose.getX(DistanceUnit.INCH);
        double robotY = robotPose.getY(DistanceUnit.INCH);
        double robotHeading = robotPose.getHeading(AngleUnit.DEGREES);

        double dx = goalX - robotX, dy = goalY - robotY;
        double targetGlobalAngle = Math.toDegrees(Math.atan2(dy, dx));
        double turretTarget = targetGlobalAngle - robotHeading;
        while (turretTarget > 180) turretTarget -= 360;
        while (turretTarget < -180) turretTarget += 360;
        turretToAngle(turretTarget);
    }

    public void updateConstants(){
        turretMotor.setInverted(RobotConstants.turretReversed);
        pdController.setP(RobotConstants.turretPCoeff);
        pdController.setD(RobotConstants.turretDCoeff);
    }

    public Motor getTurretMotor(){
        return turretMotor;
    }
}
