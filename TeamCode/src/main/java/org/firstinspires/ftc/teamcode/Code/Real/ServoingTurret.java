package org.firstinspires.ftc.teamcode.Code.Real;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous(name = "ServoingTurret")
public class ServoingTurret extends OpMode {

    private Limelight3A limelight3A;
    private DcMotor turretMotor;
    private BNO055IMU imu;

    private double KpTurret = 0.01;
    private double minPower = 0.05;

    private double robotYawOffset;
    private boolean firstLoop = true;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
        limelight3A.start();

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        robotYawOffset = imu.getAngularOrientation().firstAngle;
    }

    @Override
    public void loop() {
        double robotYaw = imu.getAngularOrientation().firstAngle - robotYawOffset;
        LLResult llResult = limelight3A.getLatestResult();
        double tx = 0.0;

        if (llResult != null && llResult.isValid()) {
            tx = llResult.getTx();
        }

        double targetAngle = robotYaw + tx;
        double currentTurretAngle = turretMotor.getCurrentPosition();
        double error = targetAngle - currentTurretAngle;

        double turretPower = 0;

        if (!firstLoop && Math.abs(error) > 1) {
            turretPower = KpTurret * error;
            if (Math.abs(turretPower) < minPower) {
                turretPower = Math.signum(turretPower) * minPower;
            }
            turretPower = Math.max(-1.0, Math.min(1.0, turretPower));
        }

        turretMotor.setPower(turretPower);
        firstLoop = false;

        telemetry.addData("TX", tx);
        telemetry.addData("Robot Yaw", robotYaw);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Turret Power", turretPower);
        telemetry.update();
    }
}
