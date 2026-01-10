package org.firstinspires.ftc.teamcode.JayCode.Subsystems;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.JayCode.RobotConstants;


public class IntakeSubsys extends SubsystemBase {
    private Motor intakeMotor;

    public IntakeSubsys(final HardwareMap hwMap){
        intakeMotor = new Motor(hwMap, RobotConstants.intakeName, Motor.GoBILDA.RPM_1150)
                .setInverted(RobotConstants.intakeReversed);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
    }

    public void runIntake(double power){
        intakeMotor.set(power);
    }

    public Motor getMotor(){
        return intakeMotor;
    }

    public void updateConstants(){
        intakeMotor.setInverted(RobotConstants.intakeReversed);
    }
}
