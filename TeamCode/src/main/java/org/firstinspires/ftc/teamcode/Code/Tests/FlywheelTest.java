package org.firstinspires.ftc.teamcode.Code.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.FlyWheel;

@TeleOp(name = "FlyWheel Test")
public class FlywheelTest extends LinearOpMode {

    private FlyWheel flywheel;

    @Override
    public void runOpMode() {

        flywheel = new FlyWheel(hardwareMap, "flywheel");

        waitForStart();

        while (opModeIsActive()) {
            double power = gamepad1.right_trigger;
            flywheel.setPower(power);

            if (gamepad1.left_trigger > 0.1) {
                flywheel.stop();
            }

            // Telemetry
            telemetry.addData("Flywheel Power", power);
            telemetry.update();
        }
    }
}
