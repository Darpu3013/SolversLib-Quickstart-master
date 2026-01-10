package org.firstinspires.ftc.teamcode.JayCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.LocalizationSubsys;

@TeleOp
public class MegatagTest extends OpMode {
    public LocalizationSubsys localizer;

    @Override
    public void init() {
        localizer = new LocalizationSubsys(hardwareMap, RobotConstants.centerPose);
    }

    @Override
    public void loop() {
        telemetry.addData("distance from goal", localizer.getDistance());
        telemetry.addData("heading", localizer.getPinpointHeading());
        telemetry.update();
    }
}
