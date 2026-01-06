package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

public class FlyWheel {

    private Motor leftFlywheel;
    private Motor rightFlywheel;
    private MotorGroup flywheelMotors;

    public FlyWheel(final HardwareMap hMap, final String name) {

        leftFlywheel = new Motor(hMap, "leftFlywheel");
        rightFlywheel = new Motor(hMap, "rightFlywheel");

        leftFlywheel.setInverted(true);
        rightFlywheel.setInverted(false);

        flywheelMotors = new MotorGroup(leftFlywheel, rightFlywheel);

        flywheelMotors.setRunMode(Motor.RunMode.VelocityControl);
        flywheelMotors.setVeloCoefficients(0, 0, 0);

    }

    public void setPower(double power) {
        flywheelMotors.set(power);
    }

    public void stop() {
        flywheelMotors.stopMotor();
    }

    public double getVelocity() {
        return flywheelMotors.getVelocity();
    }


}

