package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;


@Configurable

public class FlyWheelNew {

    private final MotorGroup flywheelMotors;

    public static double kP = -0.000001, kI = 0, kD = 0;

    public static double kS = 0, kV = -0.000401, kA = 0;

    public FlyWheelNew(final HardwareMap hMap, final String name) {

        Motor leftFlywheel = new Motor(hMap, "leftFlywheel");
        Motor rightFlywheel = new Motor(hMap, "rightFlywheel");

        leftFlywheel.setInverted(false);
        rightFlywheel.setInverted(true);

        flywheelMotors = new MotorGroup(leftFlywheel, rightFlywheel);

        flywheelMotors.setRunMode(Motor.RunMode.VelocityControl);

        flywheelMotors.setVeloCoefficients(kP, kI, kD);

        flywheelMotors.setFeedforwardCoefficients(kS, kV, kA);

    }

    public void setVelocity(double ticksPerSecond) {
        flywheelMotors.set(ticksPerSecond);
    }

    public void updateCoefficients() {
        flywheelMotors.setVeloCoefficients(kP, kI, kD);
        flywheelMotors.setFeedforwardCoefficients(kS, kV, kA);
    }

    public double getVelocity() {
        return flywheelMotors.getVelocity();
    }

    public void stop() {
        flywheelMotors.stopMotor();
    }
}
