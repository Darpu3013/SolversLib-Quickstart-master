package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class TurretNew extends SubsystemBase {

    private final Motor turretMotor;

    private static final double ENCODER_PPR = 141.1;
    private static final double GEAR_RATIO = 5.2;
    private static final double DEGREES_PER_TICK = 360.0 / ENCODER_PPR;

    private static final double MIN_ANGLE = -200;
    private static final double MAX_ANGLE = 200;

    public TurretNew(HardwareMap hMap, String name) {
        turretMotor = new Motor(hMap, name);
        turretMotor.setInverted(true);
        turretMotor.setRunMode(Motor.RunMode.PositionControl);

        turretMotor.setPositionCoefficient(0.07);
        turretMotor.resetEncoder();
    }

    public void setTurretAngle(double angleDeg) {
        angleDeg = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleDeg));
        int ticks = (int) Math.round(angleDeg / DEGREES_PER_TICK);
        turretMotor.setTargetPosition(ticks);
        turretMotor.set(0.5);
    }

    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() * DEGREES_PER_TICK;
    }

    public void stop() {
        turretMotor.stopMotor();
    }
}
