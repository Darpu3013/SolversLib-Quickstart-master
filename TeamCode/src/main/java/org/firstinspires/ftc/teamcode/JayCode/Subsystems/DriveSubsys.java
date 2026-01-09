package org.firstinspires.ftc.teamcode.JayCode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.JayCode.RobotConstants;

public class DriveSubsys extends SubsystemBase {
    private Motor flMotor, blMotor, brMotor, frMotor;

    public DriveSubsys(final HardwareMap hwMap){
        flMotor = new Motor(hwMap, RobotConstants.flName, Motor.GoBILDA.RPM_435)
                .setInverted(RobotConstants.leftDTReversed);
        blMotor = new Motor(hwMap, RobotConstants.blName, Motor.GoBILDA.RPM_435)
                .setInverted(RobotConstants.leftDTReversed);
        brMotor = new Motor(hwMap, RobotConstants.brName, Motor.GoBILDA.RPM_435)
                .setInverted(!RobotConstants.leftDTReversed);
        frMotor = new Motor(hwMap, RobotConstants.frName, Motor.GoBILDA.RPM_435)
                .setInverted(!RobotConstants.leftDTReversed);
    }

    public void runDrive(double translateY, double strafeX, double rotation, double heading){
        double y = -translateY, x = strafeX, rx = rotation, rotX, rotY;
        if(RobotConstants.fieldCentric){
             rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
             rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        } else {
            rotX = x;
            rotY = y;
        }
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = ((rotY + rotX + rx) / denominator) * RobotConstants.maxDriveSpeed;
        double backLeftPower = ((rotY - rotX + rx) / denominator) * RobotConstants.maxDriveSpeed;
        double frontRightPower = ((rotY - rotX - rx) / denominator) * RobotConstants.maxDriveSpeed;
        double backRightPower = ((rotY + rotX - rx) / denominator) * RobotConstants.maxDriveSpeed;

        flMotor.set(frontLeftPower);
        blMotor.set(backLeftPower);
        brMotor.set(backRightPower);
        frMotor.set(frontRightPower);
    }

    public void runDrive(double translateY, double strafeX, double rotation){
        double y = -translateY, x = strafeX, rx = rotation;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator) * RobotConstants.maxDriveSpeed;
        double backLeftPower = ((y - x + rx) / denominator) * RobotConstants.maxDriveSpeed;
        double frontRightPower = ((y - x - rx) / denominator) * RobotConstants.maxDriveSpeed;
        double backRightPower = ((y + x - rx) / denominator) * RobotConstants.maxDriveSpeed;

        flMotor.set(frontLeftPower);
        blMotor.set(backLeftPower);
        brMotor.set(backRightPower);
        frMotor.set(frontRightPower);
    }
}
