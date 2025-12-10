package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
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
        Logger.recordOutput("Vision/EstimatedPose", limelight.pose);
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
