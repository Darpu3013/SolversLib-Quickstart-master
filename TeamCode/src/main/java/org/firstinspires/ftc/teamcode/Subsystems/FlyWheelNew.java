package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

public class FlyWheelNew {

    private final MotorGroup flywheelMotors;

    public FlyWheelNew(final HardwareMap hMap, final String name) {

        Motor leftFlywheel = new Motor(hMap, "leftFlywheel");
        Motor rightFlywheel = new Motor(hMap, "rightFlywheel");

        // One motor must be inverted so that mirrored wheels spin the same way
        leftFlywheel.setInverted(true);
        rightFlywheel.setInverted(false);

        flywheelMotors = new MotorGroup(leftFlywheel, rightFlywheel);

        // Set velocity control mode
        flywheelMotors.setRunMode(Motor.RunMode.VelocityControl);

        // TEMP PIDF values — tune kF for your max TPS
        flywheelMotors.setVeloCoefficients(0, 0, 0);
    }

    /** Set target velocity in encoder ticks per second */
    public void setVelocity(double ticksPerSecond) {
        flywheelMotors.set(ticksPerSecond);
    }

    /** Return absolute velocity (TPS) for telemetry, handles mirrored motors */
    public double getVelocity() {
        return Math.abs(flywheelMotors.getVelocity());
    }

    /** Stop both motors */
    public void stop() {
        flywheelMotors.stopMotor();
    }
}
