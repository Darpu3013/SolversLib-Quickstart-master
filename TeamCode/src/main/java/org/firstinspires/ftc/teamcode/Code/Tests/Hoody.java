package org.firstinspires.ftc.teamcode.Code.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Hoody")
public class Hoody extends OpMode {

    private Servo hoodServo;

    @Override
    public void init() {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        hoodServo.setPosition(0.0);

        telemetry.addLine("Servo Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        hoodServo.setPosition(1.0);

        telemetry.addData("Servo Position", hoodServo.getPosition());
        telemetry.update();

    }
}
