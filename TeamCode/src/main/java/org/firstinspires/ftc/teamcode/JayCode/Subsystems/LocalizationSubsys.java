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

    private double goalX = RobotConstants.goalX, goalY;

    public LocalizationSubsys(final HardwareMap hwMap){
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, RobotConstants.pinpointName);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(1.375, 5.875, DistanceUnit.INCH);
        pinpoint.setPosition(RobotConstants.robotPose);

        limelight = hwMap.get(Limelight3A.class, RobotConstants.limelightName);
        limelight.start();
        if (RobotConstants.robotTeam == RobotConstants.Team.RED) {
            limelight.pipelineSwitch(RobotConstants.redPipeline);
            goalY = RobotConstants.redGoalY;
        } else {
            limelight.pipelineSwitch(RobotConstants.bluePipeline);
            goalY = RobotConstants.blueGoalY;
        }

    }

    public double getDistance() {
        pinpoint.update();
        double referenceX = pinpoint.getPosX(DistanceUnit.INCH);
        double referenceY = pinpoint.getPosY(DistanceUnit.INCH);
        double llX = 0, llY = 0;
        latestResult = limelight.getLatestResult();
        if (latestResult != null && latestResult.isValid()){
            Pose3D botpose = latestResult.getBotpose();
            if (botpose != null){
                llX = botpose.getPosition().x * 100 / 2.54;
                llY = botpose.getPosition().y * 100 / 2.54;
            }
        }
        if (Math.abs(referenceX) > 10 && Math.abs(referenceY) > 10){
            if (Math.signum(referenceX) == -Math.signum(llX)) {llX *= -1;}
            if (Math.signum(referenceY) == -Math.signum(llY)) {llY *= -1;}
        }
        double distX = llX - goalX, distY = llY - goalY;
        return Math.hypot(distX, distY);
    }

    public double getPinpointHeading(){
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }
}