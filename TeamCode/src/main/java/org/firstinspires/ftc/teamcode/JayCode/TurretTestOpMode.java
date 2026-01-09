package org.firstinspires.ftc.teamcode.JayCode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.LocalizationSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.TurretSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.TurretSubsystem;

@TeleOp
public class TurretTestOpMode extends OpMode {
    public TurretSubsystem oldTurret;
    public TurretSubsys turretSubsystem;
    public LocalizationSubsys localizer;
    @Override
    public void init() {
        turretSubsystem = new TurretSubsys(hardwareMap);
        localizer = new LocalizationSubsys(hardwareMap);
        oldTurret = new TurretSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        localizer.updatePinpoint();
        Pose2D pose = localizer.getPinpointPose();
        turretSubsystem.turretTrack(pose);
        //turretSubsystem.updateConstants();
        telemetry.addData("X", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heang", pose.getHeading(AngleUnit.DEGREES));
    }
}
