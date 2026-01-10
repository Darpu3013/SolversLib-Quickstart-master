package org.firstinspires.ftc.teamcode.Code.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.FlyWheelNew;

@Configurable
@TeleOp(name = "TPS Stuff", group = "Test")
public class TPSStuff extends LinearOpMode {

    private FlyWheelNew flywheel;

    public static double targetTPS = 0;
    public static double TPS_STEP = 20;

    private boolean prevUp = false;
    private boolean prevDown = false;

    @Override
    public void runOpMode() {

        flywheel = new FlyWheelNew(hardwareMap, "flywheel");

        telemetry.addLine("D-Pad Up: TPS +");
        telemetry.addLine("D-Pad Down: TPS -");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            // Increment / decrement (edge-detected)
            if (up && !prevUp) {
                targetTPS += TPS_STEP;
            }

            if (down && !prevDown) {
                targetTPS = Math.max(0, targetTPS - TPS_STEP);
            }

            prevUp = up;
            prevDown = down;

            flywheel.setVelocity(targetTPS);

            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Actual TPS", flywheel.getVelocity());
            telemetry.update();
        }

        flywheel.stop();
    }
}
