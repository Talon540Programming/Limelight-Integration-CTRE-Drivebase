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
    private final VisionIOInputs input = new VisionIOInputs();
    private final CommandSwerveDrivetrain drivetrain;


    public VisionBase(VisionIO vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
    }


    @Override
    public void periodic(){

    vision.updateLimelightYaw(drivetrain);
    vision.updateVisionIOInputs(input);

    if(input.seenTagCount > 0){
        drivetrain.addVisionMeasurement(input.pose, input.timestamp);
    }

    SmartDashboard.putBoolean("Vision/HasTarget", input.hasTarget);
        SmartDashboard.putNumber("Vision/TX", input.limelightTX);
        SmartDashboard.putNumber("Vision/TY", input.limelightTY);
        SmartDashboard.putNumber("Vision/TA", input.limelightTA);
        SmartDashboard.putNumber("Vision/TagCount", input.seenTagCount);
        SmartDashboard.putBoolean("Vision/IsRedAlliance", input.isRedAlliance);
    }

    public VisionIOInputs getVisionIOInputs(){
        return input;
    }

    public boolean hasTarget() {
        return input.hasTarget;
    }
    
    public double getTX() {
        return input.limelightTX;
    }
    
    public boolean isRedAlliance() {
        return input.isRedAlliance;
    }


}
