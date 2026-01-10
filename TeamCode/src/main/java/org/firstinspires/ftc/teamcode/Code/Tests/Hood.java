package org.firstinspires.ftc.teamcode.Code.Tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "Hood Test")
public class Hood extends OpMode {

    private Limelight3A limelight;
    private Servo hoodServo;

    // Hood servo mapping
    private static final double HOOD_MIN_ANGLE = 30.0; // degrees
    private static final double HOOD_MAX_ANGLE = 60.0; // degrees

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        // Set hood to minimum at start
        hoodServo.setPosition(0.08);


        limelight.pipelineSwitch(2); // AprilTag pipeline
        limelight.start();

        telemetry.addLine("Limelight Hood Control Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ta = result.getTa();

            // Calculate distance from ta using trendline
            double distanceInches = ta > 0 ? 68.8 * Math.pow(ta, -0.5) : 0;

            // Clamp ratio to avoid NaN
            double ratio = Math.min(distanceInches / 159.0, 1.0);
            double hoodAngle = Math.toDegrees(Math.asin(ratio));

            // Map hood angle to servo position
            double servoPosition = (hoodAngle - HOOD_MIN_ANGLE) / (HOOD_MAX_ANGLE - HOOD_MIN_ANGLE);
            servoPosition = Math.max(0.0754, Math.min(0.08, servoPosition)); // clamp

            // Set servo position
            hoodServo.setPosition(servoPosition);

            telemetry.addLine("TARGET DETECTED");
            telemetry.addData("Raw ta", "%.3f", ta);
            telemetry.addData("Distance (inches)", "%.2f", distanceInches);
            telemetry.addData("Hood Angle (degrees)", "%.2f", hoodAngle);
            telemetry.addData("Servo Position", "%.2f", servoPosition);

        } else {
            telemetry.addLine("NO TARGET");
        }

        telemetry.addLine();
        telemetry.addLine("Hood moves automatically based on Limelight distance");
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
