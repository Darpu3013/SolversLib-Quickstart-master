package org.firstinspires.ftc.teamcode.Code.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.FlyWheelNew;

@Configurable
//@TeleOp(name = "Flywheel TPS Tuner", group = "Test")
public class FlyWheelTPSTuner extends LinearOpMode {

    private FlyWheelNew flywheel;
    private Servo hoodServo;

    private DcMotor intakeMotor;

    public static double targetTPS = 0;
    private static final double TPS_STEP = 20;

    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevLB = false;
    private boolean prevRB = false;

    @Override
    public void runOpMode() {

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        flywheel = new FlyWheelNew(hardwareMap, "flywheel");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Flywheel TPS Tuner Ready");
        telemetry.addLine("D-Pad Up/Down: Adjust TPS");
        telemetry.addLine("A: Stop");
        telemetry.addLine("LB/RB: Adjust Hood");
        telemetry.update();

        waitForStart();

        hoodServo.setPosition(0.5);

        while (opModeIsActive()) {

            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            // TPS tuning (edge-detected)
            if (up && !prevUp) {
                targetTPS += TPS_STEP;
            }

            if (down && !prevDown) {
                targetTPS = Math.max(0, targetTPS - TPS_STEP);
            }

            if (gamepad1.a) {
                targetTPS = 0;
            }

            // Hood tuning (edge-detected + clamped)
            if (gamepad1.rightBumperWasPressed()) {
                hoodServo.setPosition(hoodServo.getPosition() + 0.02);
            }

            if (gamepad1.leftBumperWasPressed()) {
                hoodServo.setPosition(hoodServo.getPosition() - 0.02);
            }

            if (gamepad1.right_trigger >= 0.2) {
                intakeMotor.setPower(1);
            } else if (gamepad1.triangle) {
                intakeMotor.setPower(-0.5);
            } else {
                intakeMotor.setPower(0);
            }


            prevUp = up;
            prevDown = down;
            prevLB = lb;
            prevRB = rb;

            flywheel.setVelocity(targetTPS);
            flywheel.updateCoefficients();

            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Actual TPS", flywheel.getVelocity());
            telemetry.addData("Hood Position", hoodServo.getPosition());
            telemetry.update();
        }

        flywheel.stop();
    }
}
