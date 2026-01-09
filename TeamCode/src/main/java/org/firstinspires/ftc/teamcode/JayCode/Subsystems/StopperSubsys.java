package org.firstinspires.ftc.teamcode.JayCode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.JayCode.RobotConstants;

public class StopperSubsys extends SubsystemBase {
    private ServoEx stopperServo;

    public StopperSubsys(final HardwareMap hwMap){
        stopperServo = new ServoEx(hwMap, RobotConstants.stopperName);
    }

    public void stopperClose(){
        stopperServo.set(RobotConstants.stopperClose);
    }
    public void stopperOpen(){
        stopperServo.set(RobotConstants.stopperOpen);
    }
}
