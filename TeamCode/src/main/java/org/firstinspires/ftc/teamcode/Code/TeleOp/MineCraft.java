package org.firstinspires.ftc.teamcode.Code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Minecraft1 (Blocks to Java)")
public class MineCraft extends LinearOpMode {

    private Servo kickerServo;
    private Servo hoodServo;
    private DcMotor leftFlywheel;
    private DcMotor flMotor;
    private DcMotor blMotor;
    private DcMotor intakeMotor;
    private DcMotor turretMotor;
    private DcMotor brMotor;
    private DcMotor frMotor;
    private DcMotor rightFlywheel;

    int hoodMinimum;

    private void BlockShooter(boolean ShooterBlocked_) {
        if (ShooterBlocked_) {
            kickerServo.setPosition(0);
        } else {
            kickerServo.setPosition(0);
        }
    }

    private void hoodToAngle(int Target_Angle) {
        hoodServo.setPosition(hoodMinimum + Target_Angle / 355.0);
    }

    @Override
    public void runOpMode() {
        int turretMaxTicks;
        int maxSpeed;
        float translate;
        float strafe;
        float pivot;

        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        leftFlywheel = hardwareMap.get(DcMotor.class, "leftFlywheel");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        rightFlywheel = hardwareMap.get(DcMotor.class, "rightFlywheel");

        hoodServo.setDirection(Servo.Direction.FORWARD);
        kickerServo.setDirection(Servo.Direction.FORWARD);

        leftFlywheel.setDirection(DcMotor.Direction.REVERSE);
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretMaxTicks = 74;
        hoodMinimum = 0;

        waitForStart();

        if (opModeIsActive()) {
            maxSpeed = 1;

            while (opModeIsActive()) {

                translate = -gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x;
                pivot = gamepad1.right_stick_x;

                // Flywheels
                if (gamepad1.left_trigger >= 0.2) {
                    rightFlywheel.setPower(1);
                    leftFlywheel.setPower(1);
                } else {
                    rightFlywheel.setPower(0);
                    leftFlywheel.setPower(0);
                }

                // Drive
                flMotor.setPower((translate + strafe + pivot) * maxSpeed);
                blMotor.setPower((translate - strafe + pivot) * maxSpeed);
                brMotor.setPower((translate + strafe - pivot) * maxSpeed);
                frMotor.setPower((translate - strafe - pivot) * maxSpeed);

                // Intake
                if (gamepad1.right_trigger >= 0.2) {
                    intakeMotor.setPower(1);
                } else if (gamepad1.triangle) {
                    intakeMotor.setPower(-0.5);
                } else {
                    intakeMotor.setPower(0);
                }

                // Turret
                if (gamepad1.right_bumper && Math.abs(turretMotor.getCurrentPosition()) <= 72) {
                    turretMotor.setPower(0.3);
                } else if (gamepad1.left_bumper && Math.abs(turretMotor.getCurrentPosition()) <= 72) {
                    turretMotor.setPower(-0.3);
                } else if (gamepad1.left_bumper && turretMotor.getCurrentPosition() > 72) {
                    turretMotor.setPower(-0.3);
                } else if (gamepad1.right_bumper && turretMotor.getCurrentPosition() < -72) {
                    turretMotor.setPower(0.3);
                } else {
                    turretMotor.setPower(0);
                }

                // Hood control
                if (gamepad1.square) {
                    hoodServo.setPosition(0.735); // SHOOT POSITION
                } else if (gamepad1.circle) {
                    hoodToAngle(0);
                } else if (gamepad1.x) {
                    hoodToAngle(0);
                }

                telemetry.addData("turretticks", turretMotor.getCurrentPosition());
                telemetry.addData("hoodPos", hoodServo.getPosition());
                telemetry.addData("kickerPos", kickerServo.getPosition());
                telemetry.update();
            }
        }
    }
}
