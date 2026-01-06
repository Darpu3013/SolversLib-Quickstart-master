package org.firstinspires.ftc.teamcode.Code.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Increment Control")
public class ServoMove extends OpMode {

    private Servo hoodServo;

    private double servoPosition = 0.0;
    private final double STEP = 0.5;

    private boolean lastLB = false;
    private boolean lastRB = false;

    @Override
    public void init() {

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        // Optional – try flipping this if movement feels backwards
        // hoodServo.setDirection(Servo.Direction.REVERSE);

        hoodServo.setPosition(servoPosition);

        telemetry.addLine("Servo Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Increase
        if (gamepad1.right_bumper && !lastRB) {
            servoPosition += STEP;
        }

        // Decrease
        if (gamepad1.left_bumper && !lastLB) {
            servoPosition -= STEP;
        }

        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;

        servoPosition = Range.clip(servoPosition, 0.0, 1.0);
        hoodServo.setPosition(servoPosition);

        telemetry.addData("Servo Position", "%.3f", servoPosition);
        telemetry.addLine("LB = down | RB = up");
        telemetry.update();
    }
}
