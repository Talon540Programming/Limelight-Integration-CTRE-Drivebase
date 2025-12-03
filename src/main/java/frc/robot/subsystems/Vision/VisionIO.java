package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {

    public static class VisionIOInputs{
        public boolean hasTarget = false;
        public double limelightTX = 0.0;
        public double limelightTY = 0.0;
        public double limelightTA = 0.0;
        public Pose2d pose = new Pose2d();
        public double timestamp = 0.0;
        public int seenTagCount = 0;
        public boolean isRedAlliance = false;
    }

    default void updateVisionIOInputs(VisionIOInputs io){}

    default void updateLimelightYaw(CommandSwerveDrivetrain drivetrain){}  
}
