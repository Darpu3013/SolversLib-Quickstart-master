package org.firstinspires.ftc.teamcode.Code.Tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight TA Collector with Distance")
public class Distance extends OpMode {

    private Limelight3A limelight;

    // Averaging
    private static final int MAX_SAMPLES = 30;
    private double taSum = 0;
    private int taCount = 0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(2);   // AprilTag pipeline
        limelight.start();

        telemetry.addLine("Limelight TA Collector Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ta = result.getTa();

            // Only collect up to MAX_SAMPLES
            if (ta > 0 && taCount < MAX_SAMPLES) {
                taSum += ta;
                taCount++;
            }

            double taAvg = taCount > 0 ? taSum / taCount : 0;

            // Calculate distance using your new trendline equation
            double distanceInches = taAvg > 0 ? 69.0 * Math.pow(taAvg, -0.506) : 0;

            telemetry.addLine("TARGET DETECTED");
            telemetry.addData("Raw ta", "%.3f", ta);
            telemetry.addData("Avg ta (frozen after 30)", "%.3f", taAvg);
            telemetry.addData("Distance (inches)", "%.2f", distanceInches);
            telemetry.addData("Samples", "%d / %d", taCount, MAX_SAMPLES);

            if (taCount >= MAX_SAMPLES) {
                telemetry.addLine("AVERAGE LOCKED");
            }
        } else {
            telemetry.addLine("NO TARGET");
        }

        telemetry.addLine();
        telemetry.addLine("Measure distance with tape measure");
        telemetry.addLine("Write down Avg ta and Distance");
        telemetry.addLine("Press X to reset average");

        // Reset averaging when you press X
        if (gamepad1.x) {
            taSum = 0;
            taCount = 0;
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
