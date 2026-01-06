package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Turret extends SubsystemBase {

    private final Motor turretMotor;
    private static final double ENCODER_PPR = 145.1;
    private static final double GEAR_RATIO = 2;
    private static final double DEGREES_PER_TICK = 360.0 / (ENCODER_PPR * GEAR_RATIO);
    private static final double MIN_ANGLE = -150;
    private static final double MAX_ANGLE = 150;

    public Turret(final HardwareMap hMap, final String name){
        turretMotor = new Motor(hMap, name);
        turretMotor.setInverted(true);
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.resetEncoder();
    }

    public void setTurretAngle(double angle) {
        angle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
        int targetTicks = (int) Math.round(angle / DEGREES_PER_TICK);
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.set(0.3);
    }

    public void setMotorPower(double power) {
        power = Math.max(-1, Math.min(1, power));
        turretMotor.set(power);

    }

    public void stop() {
        turretMotor.stopMotor();
    }

    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() * DEGREES_PER_TICK;
    }
}
