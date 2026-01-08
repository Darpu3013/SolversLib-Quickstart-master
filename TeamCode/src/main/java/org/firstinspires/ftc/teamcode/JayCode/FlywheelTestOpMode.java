package org.firstinspires.ftc.teamcode.JayCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.JayCode.Subsystems.FlywheelSubsys;

@TeleOp
public class FlywheelTestOpMode extends OpMode {
    public FlywheelSubsys flywheels;
    double velocity;

    @Override
    public void init() {
        flywheels = new FlywheelSubsys(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()){
            velocity += 20;
        }
        else if (gamepad1.dpadDownWasPressed()){
            velocity -= 20;
        }
        flywheels.setFlywheelVel(velocity);
        flywheels.updateConstants();
        telemetry.addData("target", velocity);
        telemetry.addData("actual vel", flywheels.getFlywheelVel());
        telemetry.update();
    }
}
