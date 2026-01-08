package org.firstinspires.ftc.teamcode.JayCode.Subsystems;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.JayCode.RobotConstants;

public class TurretSystem extends SubsystemBase {
    private Motor turretMotor;
    private PDController pdController;

    public TurretSystem(HardwareMap hwMap){
        this.turretMotor = new Motor(hwMap, RobotConstants.turretName, RobotConstants.turretMotType)
                .setInverted(RobotConstants.turretReversed);
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.stopAndResetEncoder();
    }

    public void turretToPosition(double target){
        double clampedTarg = MathUtils.clamp(target, RobotConstants.turretLeftMax, RobotConstants.turretRightMax);
        int goal = (int)(clampedTarg * RobotConstants.turretTicksPerDegree);
        double current = turretMotor.getCurrentPosition();
        double output = pdController.calculate(current, goal);
        turretMotor.set(output);
    }

    public Motor getTurretMotor(){
        return turretMotor;
    }
}
