package org.firstinspires.ftc.teamcode.JayCode.Subsystems;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.JayCode.RobotConstants;

public class HoodSubsys extends SubsystemBase {
    private ServoEx hoodServo;

    public HoodSubsys(final HardwareMap hwMap){
        hoodServo = new ServoEx(hwMap, RobotConstants.hoodServoName);
        hoodServo.setInverted(RobotConstants.hoodReversed);
    }

    public void hoodTo(double target){
        double clampedTarg = MathUtils.clamp(target, RobotConstants.hoodMin, RobotConstants.hoodMax);
        hoodServo.set(clampedTarg);
    }

    public ServoEx getServo(){
        return hoodServo;
    }
}
