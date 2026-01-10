package org.firstinspires.ftc.teamcode.JayCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.DriveSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.FlywheelSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.HoodSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.IntakeSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.LocalizationSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.StopperSubsys;
import org.firstinspires.ftc.teamcode.JayCode.Subsystems.TurretSubsys;

@TeleOp
public class JayTeleOpBlue extends OpMode {

    private DriveSubsys drivetrain;
    private FlywheelSubsys flywheel;
    private HoodSubsys hood;
    private IntakeSubsys intake;
    private LocalizationSubsys localizer;
    private StopperSubsys stopper;
    private TurretSubsys turret;
    public boolean flywheelOn = false;

    Pose2D latestPose;

    @Override
    public void init() {
        RobotConstants.robotTeam = RobotConstants.Team.BLUE;

        drivetrain = new DriveSubsys(hardwareMap);
        flywheel = new FlywheelSubsys(hardwareMap);
        hood = new HoodSubsys(hardwareMap);
        intake = new IntakeSubsys(hardwareMap);
        localizer = new LocalizationSubsys(hardwareMap, RobotConstants.endPose);
        stopper = new StopperSubsys(hardwareMap);
        turret = new TurretSubsys(hardwareMap);
    }

    @Override
    public void loop() {
        localizer.updatePinpoint();
        latestPose = localizer.getPinpointPose();
        double distanceToGoal = localizer.getDistance();

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rot = gamepad1.right_stick_x;
        double heading = localizer.getPinpointHeading();
        drivetrain.runDrive(y, x, rot, heading);

        hood.runHoodRegression(distanceToGoal);

        if (gamepad1.rightBumperWasPressed()){
            flywheelOn = !flywheelOn;
        }
        if (flywheelOn){
            if (stopper.getPos() != RobotConstants.stopperOpen){
                stopper.stopperOpen();
            }
            flywheel.runFlywheelRegression(distanceToGoal);
        } else {
            flywheel.idleFlywheel();
            if (stopper.getPos() != RobotConstants.stopperClose) {
                stopper.stopperClose();
            }
        }

        turret.turretTrack(latestPose);

        if (gamepad1.right_trigger >= 0.1){
            intake.runIntake(1);
        } else if (gamepad1.left_trigger >= 0.2){
            intake.runIntake(-gamepad1.left_trigger);
        } else {
            intake .runIntake(0);
        }

        telemetry.addData("hoodpos", hood.getHoodOutput(distanceToGoal));
        telemetry.update();
    }
}
