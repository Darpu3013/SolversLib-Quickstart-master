package org.firstinspires.ftc.teamcode.JayCode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.TurretSubsystem;

@TeleOp
public class TurretTestOpMode extends OpMode {
    public TurretSubsystem turretSubsystem;

    @Override
    public void init() {
        turretSubsystem = new TurretSubsystem(hardwareMap);
        turretSubsystem.pinpoint.resetPosAndIMU();
        turretSubsystem.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        turretSubsystem.pinpoint.setOffsets(1.375, 5.875, DistanceUnit.INCH);
    }

    @Override
    public void loop() {
        turretSubsystem.runTurret();
        turretSubsystem.updateConstants();
        telemetry.addData("X", turretSubsystem.pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y", turretSubsystem.pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Heang", turretSubsystem.pinpoint.getHeading(AngleUnit.DEGREES));
    }
}
