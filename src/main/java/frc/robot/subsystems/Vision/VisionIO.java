package frc.robot.subsystems.Vision;


public interface VisionIO {
    public static class VisionInputs{
        public boolean isRedAlliance;
    }
    default boolean getTV(){
        return false;
    }
    default double getTX(){
        return 0.0;
    }
    default double getTA(){
        return 0.0;
    }

    default boolean isRedAlliance(){
        return true;
    }

    default void updatePoseEstimatorMT2(CommandSwerveDrivetrain drivetrain){}

    default void updateLimelightYaw(CommandSwerveDrivetrain drivetrain){}  
}
