package org.firstinspires.ftc.teamcode.JayCode.Subsystems;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.JayCode.RobotConstants;

public class TurretSubsystem extends SubsystemBase {

    private Motor turretMotor;
    public GoBildaPinpointDriver pinpoint;
    private PDController pd;

    public TurretSubsystem(final HardwareMap hwMap){
        turretMotor = new Motor(hwMap, RobotConstants.turretName, RobotConstants.turretMotType)
                .setInverted(RobotConstants.turretReversed);
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.stopAndResetEncoder();
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(1.375, 5.875, DistanceUnit.INCH);
    }

    public void turretToDegrees(double targetDegrees){
        double clampedTarget = MathUtils.clamp(targetDegrees, RobotConstants.turretLeftMax, RobotConstants.turretRightMax);
        int goal = (int)(clampedTarget * RobotConstants.turretTicksPerDegree);
        double current = turretMotor.getCurrentPosition();
        double output = pd.calculate(current, goal);
        turretMotor.set(output);
    }

    public void runTurret(){
        pinpoint.update();
        double dx = RobotConstants.blueGoalX - pinpoint.getPosX(DistanceUnit.INCH);
        double dy = RobotConstants.blueGoalY - pinpoint.getPosY(DistanceUnit.INCH);
        double targetGlobalAngle = Math.toDegrees(Math.atan2(dy, dx));
        double turretTarget = targetGlobalAngle - pinpoint.getHeading(AngleUnit.DEGREES);
        if (turretTarget > 180) turretTarget -= 360;
        if (turretTarget < -180) turretTarget += 360;
        turretToDegrees(turretTarget);
    }

    public void updateConstants(){
        pd.setP(RobotConstants.turretPCoeff);
        pd.setD(RobotConstants.turretDCoeff);
        turretMotor.setInverted(RobotConstants.turretReversed);
    }

    public double getCurrentPos(){
        return turretMotor.getCurrentPosition();
    }
}
