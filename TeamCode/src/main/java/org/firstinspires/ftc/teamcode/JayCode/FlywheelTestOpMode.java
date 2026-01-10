package org.firstinspires.ftc.teamcode.JayCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.JayCode.Subsystems.FlywheelSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.HoodSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.LocalizationSubsys;

@TeleOp
public class FlywheelTestOpMode extends OpMode {
    public FlywheelSubsys flywheels;
    public HoodSubsys hood;
    public LocalizationSubsys localizator;
    double velocity;
    public Motor intake;

    @Override
    public void init() {
        flywheels = new FlywheelSubsys(hardwareMap);
        hood = new HoodSubsys(hardwareMap);
        localizator = new LocalizationSubsys(hardwareMap, RobotConstants.centerPose);
        hood.hoodTo(0.5);
        intake = new Motor(hardwareMap, "intakeMotor");
        intake.setInverted(true);
    }

    @Override
    public void loop() {
        /*if (gamepad1.dpadUpWasPressed()) {
            velocity += 20;
        } else if (gamepad1.dpadDownWasPressed()) {
            velocity -= 20;
        }
        if (gamepad1.rightBumperWasPressed()) {
            hood.hoodTo(hood.getServo().getRawPosition() + 0.02);
        }
        if (gamepad1.leftBumperWasPressed()) {
            hood.hoodTo(hood.getServo().getRawPosition() - 0.02);}*/
        intake.set(gamepad1.right_trigger);

        double distance = localizator.getDistance();
        flywheels.runFlywheelRegression(distance);
        hood.runHoodRegression(distance);
        flywheels.updateConstants();
        telemetry.addData("target", velocity);
        telemetry.addData("hoodPos", hood.getServo().getRawPosition());
        telemetry.addData("distance", localizator.getDistance());
        telemetry.addData("hoodTarget", hood.getHoodOutput(distance));
        telemetry.update();
        }
    }

