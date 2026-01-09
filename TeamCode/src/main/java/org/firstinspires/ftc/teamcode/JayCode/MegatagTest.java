package org.firstinspires.ftc.teamcode.JayCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.JayCode.Subsystems.LocalizationSubsys;

@TeleOp
public class MegatagTest extends OpMode {
    public LocalizationSubsys localizer;

    @Override
    public void init() {
        localizer = new LocalizationSubsys(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("robot x", localizer.getRobotPos().getPosition().x);
        telemetry.addData("robot y", localizer.getRobotPos().getPosition().y);
        telemetry.update();
    }
}
