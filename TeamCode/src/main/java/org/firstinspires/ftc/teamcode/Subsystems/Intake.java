package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final DcMotor intakeMotor;


    public Intake(final HardwareMap hMap, final String name){
        intakeMotor = hMap.get(DcMotor.class, name);
    }

    public void intake() {
        intakeMotor.setPower(1);
    }

    public void outtake() {
        intakeMotor.setPower(-1);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
