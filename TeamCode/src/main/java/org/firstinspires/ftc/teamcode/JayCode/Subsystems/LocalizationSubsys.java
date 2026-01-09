package org.firstinspires.ftc.teamcode.JayCode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.JayCode.RobotConstants;

public class LocalizationSubsys extends SubsystemBase {
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    private LLResult latestResult;

    public LocalizationSubsys(final HardwareMap hwMap){
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, RobotConstants.pinpointName);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(1.375, 5.875, DistanceUnit.INCH);
        pinpoint.setPosition(RobotConstants.robotPose);

        limelight = hwMap.get(Limelight3A.class, RobotConstants.limelightName);
        limelight.start();
        if (RobotConstants.robotTeam == RobotConstants.Team.Red) {
            limelight.pipelineSwitch(RobotConstants.redPipeline);
        } else {
            limelight.pipelineSwitch(RobotConstants.bluePipeline);
        }
    }

    public Pose3D getRobotPos(){
        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);
        latestResult = limelight.getLatestResult();
        limelight.updateRobotOrientation(heading);
        if (latestResult != null && latestResult.isValid()){
            Pose3D botPose = latestResult.getBotpose_MT2();
            if (botPose != null){
                RobotConstants.limelightPose = botPose;
            }
        }
        return RobotConstants.limelightPose;
    }

    public double getDistance(){
        return 0;
    }

    public double getPinpointHeading(){
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }
}
