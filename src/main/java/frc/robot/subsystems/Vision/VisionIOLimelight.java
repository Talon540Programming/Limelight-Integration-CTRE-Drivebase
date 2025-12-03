package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
    private final String limelightName = Constants.kLimelightName;

    public VisionIOLimelight(){
        //sets mode for Metatag 2 and mode 0 uses robot gyro for orentation 
        LimelightHelpers.SetIMUMode(limelightName, 0);
    }

    @Override
    public boolean getTV(){
        return LimelightHelpers.getTV(limelightName);
    }

    @Override
    public double getTX(){
        return LimelightHelpers.getTX(limelightName);
    }

    @Override
    public double getTA(){
        return LimelightHelpers.getTA(limelightName);
    }

    @Override
    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
    
    //will work once CTRE code is put in
    @Override
    public void updatePoseEstimatorMT2(CommandSwerveDrivetrain drivetrain){
        //stores the current pose estimate from the limelight
        LimelightHelpers.PoseEstimate metaTag2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        //adds the stored value of limelight pose estimation to odometry if tag is seen
        if(metaTag2Pose != null){
            if(metaTag2Pose.tagCount > 0){
                drivetrain.addVisionMeasurement(metaTag2Pose.pose, metaTag2Pose.timestampSeconds);
            }
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
