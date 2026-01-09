package org.firstinspires.ftc.teamcode.JayCode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.JayCode.RobotConstants;

public class FlywheelSubsys extends SubsystemBase {
    private MotorGroup flywheelMotors;
    private Motor leftFlywheel;
    private Motor rightFlywheel;

    public FlywheelSubsys(final HardwareMap hwMap){
        leftFlywheel = new Motor(hwMap, RobotConstants.leftFlywheelName, Motor.GoBILDA.BARE)
                .setInverted(RobotConstants.leftFlywheelInverted);
        leftFlywheel.setRunMode(Motor.RunMode.VelocityControl);
        leftFlywheel.stopAndResetEncoder();
        leftFlywheel.setVeloCoefficients(RobotConstants.flywheelPCoeff, 0, RobotConstants.flywheelDCoeff);
        leftFlywheel.setFeedforwardCoefficients(0, RobotConstants.flywheelKV);

        rightFlywheel = new Motor(hwMap, RobotConstants.rightFlyWheelName, Motor.GoBILDA.BARE)
                .setInverted(!RobotConstants.leftFlywheelInverted);
        flywheelMotors = new MotorGroup(leftFlywheel, rightFlywheel);
    }

    public void setFlywheelVel(double velocity){
        flywheelMotors.set(velocity);
    }

    public void runFlywheelRegression(double distance){
        double vel = 0.0103226 * Math.pow(distance, 2) + 5.71055 * distance + 1639.80756;
        setFlywheelVel(vel);
    }

    public void updateConstants(){
        flywheelMotors.setVeloCoefficients(RobotConstants.flywheelPCoeff, 0, RobotConstants.flywheelDCoeff);
        flywheelMotors.setFeedforwardCoefficients(0, RobotConstants.flywheelKV);
    }

    public double getFlywheelVel(){
        return flywheelMotors.getVelocity();
    }
}