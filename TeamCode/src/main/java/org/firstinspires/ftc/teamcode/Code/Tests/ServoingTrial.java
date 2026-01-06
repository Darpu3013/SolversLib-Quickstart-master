package org.firstinspires.ftc.teamcode.Code.Tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.bosch.BNO055IMU;


@Autonomous(name = "ServoingTrial")
public class ServoingTrial extends OpMode {

    private Limelight3A limelight3A;
    private DcMotor turretMotor;
    private BNO055IMU imu;

    private double KpTurret = 0.03;
    private double minPower = 0.03;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
        limelight3A.start();

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    @Override
    public void loop() {

        // Get current robot heading
        double robotYaw = imu.getAngularOrientation().firstAngle; // degrees

        LLResult llResult = limelight3A.getLatestResult();
        double tx = 0.0;

        if (llResult != null && llResult.isValid()) {
            tx = llResult.getTx(); // angle from camera
        }

        // Compute desired turret angle (field-centric)
        double targetAngle = robotYaw + tx;

        // If your turret has encoder:
        double currentTurretAngle = turretMotor.getCurrentPosition();
        // TODO: convert encoder ticks to degrees if needed

        double error = targetAngle - currentTurretAngle;

        // Simple proportional control
        double turretPower = 0;
        if (Math.abs(error) > 1) { // deadzone
            turretPower = KpTurret * error;
            if (Math.abs(turretPower) < minPower) {
                turretPower = Math.signum(turretPower) * minPower;
            }
            turretPower = Math.max(-1.0, Math.min(1.0, turretPower));
        }

        turretMotor.setPower(turretPower);

        telemetry.addData("TX", tx);
        telemetry.addData("Robot Yaw", robotYaw);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Turret Power", turretPower);
        telemetry.update();
    }
}
