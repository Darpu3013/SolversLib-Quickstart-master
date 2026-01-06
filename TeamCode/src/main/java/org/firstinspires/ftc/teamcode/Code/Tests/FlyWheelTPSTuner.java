package org.firstinspires.ftc.teamcode.Code.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.FlyWheelNew;

@TeleOp(name = "Flywheel TPS Tuner", group = "Test")
public class FlyWheelTPSTuner extends LinearOpMode {

    private FlyWheelNew flywheel;

    private double targetTPS = 0;
    private static final double TPS_STEP = 20;

    private boolean prevUp = false;
    private boolean prevDown = false;

    @Override
    public void runOpMode() {

        flywheel = new FlyWheelNew(hardwareMap, "flywheel");

        telemetry.addLine("Flywheel TPS Tuner Ready");
        telemetry.addLine("D-Pad Up/Down: Adjust TPS");
        telemetry.addLine("A: Stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            // Increase TPS (one step per press)
            if (up && !prevUp) {
                targetTPS += TPS_STEP;
            }

            // Decrease TPS (clamped to 0)
            if (down && !prevDown) {
                targetTPS = Math.max(0, targetTPS - TPS_STEP);
            }

            // Stop flywheel
            if (gamepad1.a) {
                targetTPS = 0;
            }

            prevUp = up;
            prevDown = down;

            // Set flywheel speed
            flywheel.setVelocity(targetTPS);

            // Telemetry shows positive TPS
            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Actual TPS", flywheel.getVelocity());
            telemetry.update();
        }

        flywheel.stop();
    }
}
