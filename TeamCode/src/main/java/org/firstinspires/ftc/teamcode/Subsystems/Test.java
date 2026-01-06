package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp(name = "Turret Test TeleOp")
public class Test extends LinearOpMode {

    private Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize turret (name must match your configuration)
        turret = new Turret(hardwareMap, "turretMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Map joystick input to turret angle
            // Assume left stick x ranges from -1 to 1 → map to -90° to 90°
            double joystickX = gamepad1.left_stick_x;
            double targetAngle = joystickX * 150; // -90 to 90

            turret.setTurretAngle(targetAngle);

            // Telemetry
            telemetry.addData("Joystick X", joystickX);
            telemetry.addData("Turret Angle (deg)", turret.getTurretAngle());
            telemetry.update();
        }

        // Stop turret at the end
        turret.stop();
    }
}
