package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;

public class VisionBase extends SubsystemBase{

    private final VisionIO vision;
    private final VisionIOInputs limelight = new VisionIOInputs();
    private final CommandSwerveDrivetrain drivetrain;


    public VisionBase(VisionIO vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic(){

    vision.updateLimelightYaw(drivetrain);
    vision.updateVisionIOInputs(limelight);

    if(limelight.seenTagCount > 0 && limelight.hasTarget){
        drivetrain.addVisionMeasurement(limelight.pose, limelight.limelightTimestamp);
    }

    SmartDashboard.putBoolean("Vision/HasTarget", limelight.hasTarget);
        SmartDashboard.putNumber("Vision/TX", limelight.limelightTX);
        SmartDashboard.putNumber("Vision/TY", limelight.limelightTY);
        SmartDashboard.putNumber("Vision/TA", limelight.limelightTA);
        SmartDashboard.putNumber("Vision/TagCount", limelight.seenTagCount);
        SmartDashboard.putBoolean("Vision/IsRedAlliance", limelight.isRedAlliance);
    }

    public VisionIOInputs getVisionIOInputs(){
        return limelight;
    }

    public boolean hasTarget() {
        return limelight.hasTarget;
    }
    
    public double getTX() {
        return limelight.limelightTX;
    }
    
    public boolean isRedAlliance() {
        return limelight.isRedAlliance;
    }
}
