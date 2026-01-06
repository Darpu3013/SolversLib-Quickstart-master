package org.firstinspires.ftc.teamcode.Subsystems;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeUpdated extends SubsystemBase {

    private final DcMotor intakeMotor;

    private int startTicks = 0;
    private boolean feeding = false;

    // TICKS_PER_3_BALLS = approx 2400
    private static final int TICKS_PER_3_BALLS = 2400;

    public IntakeUpdated(final HardwareMap hMap, final String name){
        intakeMotor = hMap.get(DcMotor.class, name);

        // Reset encoder at start
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startFeeding() {
        startTicks = Math.abs(intakeMotor.getCurrentPosition());
        feeding = true;
        intakeMotor.setPower(-1.0);
    }


    public void update() {
        if (!feeding) return;

        int delta = Math.abs(intakeMotor.getCurrentPosition() - startTicks);

        if (delta >= TICKS_PER_3_BALLS) {
            stop();
        }
    }

    public void stop() {
        intakeMotor.setPower(0);
        feeding = false;
    }

    public boolean isFeeding() {
        return feeding;
    }
}
