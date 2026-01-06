package org.firstinspires.ftc.teamcode.Code.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Intake Encoder Test")
public class IntakeEncoderTest extends OpMode {

    private DcMotor intakeMotor;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Use a gamepad button to run intake
        if (gamepad1.dpad_up) {
            intakeMotor.setPower(-1.0); // intake forward
        } else if (gamepad1.dpad_down) {
            intakeMotor.setPower(1.0); // outtake
        } else {
            intakeMotor.setPower(0);
        }

        // Telemetry: show encoder value
        telemetry.addData("Intake Encoder", intakeMotor.getCurrentPosition());
        telemetry.update();
    }
}
