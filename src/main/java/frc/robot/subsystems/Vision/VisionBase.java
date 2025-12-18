package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    public void periodic() {
        vision.updateLimelightYaw(drivetrain);
        vision.updateVisionIOInputs(limelight);
    
        if (limelight.seenTagCount > 0 && limelight.hasTarget) {
            // Reject if pose is outside the field
            if (limelight.pose.getX() < 0 || limelight.pose.getX() > 17.55 ||
                limelight.pose.getY() < 0 || limelight.pose.getY() > 8.05) {
                Logger.recordOutput("Vision/Rejected", "Out of field bounds");
                return;
            }
            
            // Reject if pose jumped too far from current estimate (possible glitch)
            double poseDelta = drivetrain.getPose().getTranslation()
            .getDistance(limelight.pose.getTranslation());

            // Allow big corrections when vision is confident
            boolean visionIsConfident = limelight.seenTagCount >= 1 && limelight.avgTagDistance < 3.0;

            if (poseDelta > 1.5 && !visionIsConfident) {
                Logger.recordOutput("Vision/Rejected", "Large jump with low confidence");
                return;
            }
            
            // Reject during fast rotation (motion blur causes bad readings)
            if (Math.abs(drivetrain.getFieldVelocity().omegaRadiansPerSecond) > Math.toRadians(180)) {
                Logger.recordOutput("Vision/Rejected", "Rotating too fast");
                return;
            }
            
            Matrix<N3, N1> stdDevs = calculateStdDevs(limelight);
            drivetrain.addVisionMeasurement(limelight.pose, limelight.limelightTimestamp, stdDevs);
        }
        
        Logger.recordOutput("Vision/EstimatedPose", limelight.pose);
    }

    private Matrix<N3, N1> calculateStdDevs(VisionIOInputs input) {
    // Base standard deviations (in meters for x/y, radians for rotation)
    double xyStdDev = 0.5;  // Start somewhat untrusting
    double rotStdDev = 0.5;
    
    // More tags = more confidence
    if (input.seenTagCount >= 2) {
        xyStdDev = 0.3;
        rotStdDev = 0.3;
    }
    
    if (input.avgTagDistance > 4.0) {
        xyStdDev *= 2.0;
        rotStdDev *= 2.0;
    }
    
    return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
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
