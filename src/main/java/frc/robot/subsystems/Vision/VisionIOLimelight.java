package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class VisionIOLimelight implements VisionIO {
    private final String limelightName = Constants.kLimelightName;

    public VisionIOLimelight(){
        //sets mode for Metatag 2 and mode 0 uses robot gyro for orentation 
        LimelightHelpers.SetIMUMode(limelightName, 0);
    }

    @Override
    public void updateVisionIOInputs(VisionIOInputs input){
         input.hasTarget = LimelightHelpers.getTV(limelightName);
         input.limelightTX = LimelightHelpers.getTX(limelightName);
         input.limelightTY = LimelightHelpers.getTY(limelightName);
         input.limelightTA = LimelightHelpers.getTA(limelightName);

        var alliance = DriverStation.getAlliance();
        if(!alliance.isEmpty() && alliance.get() == DriverStation.Alliance.Red){
            input.isRedAlliance = true;
        }
        else if(!alliance.isEmpty() && alliance.get() == DriverStation.Alliance.Blue){
            input.isRedAlliance = false;
        }

        LimelightHelpers.PoseEstimate metaTag2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if(metaTag2Pose != null){
            input.pose = metaTag2Pose.pose;
            input.limelightTimestamp = metaTag2Pose.timestampSeconds;
            input.seenTagCount = metaTag2Pose.tagCount;
        }
        else{
            input.seenTagCount = 0;
        }
    }

    @Override
    public void updateLimelightYaw(CommandSwerveDrivetrain drivetrain) {
    LimelightHelpers.SetRobotOrientation(
        limelightName,
        drivetrain.getPose().getRotation().getDegrees(),
        0, 0, 0, 0, 0
    );
    }

}
