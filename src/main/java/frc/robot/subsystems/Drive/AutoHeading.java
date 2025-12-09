package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Vision.ReefCentering;
import frc.robot.subsystems.Vision.VisionBase;


public class AutoHeading extends SubsystemBase{

    private final VisionBase vision;

    private boolean autoHeadingEnabled = false;
    private Rotation2d targetHeading = new Rotation2d();
    private Pose2d nearestReefFace = new Pose2d();

    public AutoHeading(VisionBase vision){
        this.vision = vision;
    }

    public void toggleAutoHeading(){
        autoHeadingEnabled = !autoHeadingEnabled;
        Logger.recordOutput("AutoHeading/Enabled", autoHeadingEnabled);
    }

    public void enableAutoHeading(){
        autoHeadingEnabled = true;
        Logger.recordOutput("AutoHeading/Enabled", autoHeadingEnabled);
    }

    public void disableAutoHeading(){
        autoHeadingEnabled = false;
        Logger.recordOutput("AutoHeading/Enabled", autoHeadingEnabled);
    }

    public boolean isEnabled(){
        return autoHeadingEnabled;
    }

    public void updateTargetHeading(Pose2d currentPose){
        nearestReefFace = currentPose.nearest(
            vision.isRedAlliance() ? FieldPoses.redReefPoses : FieldPoses.blueReefPoses
        );

        targetHeading = nearestReefFace.getRotation().plus(Rotation2d.fromDegrees(180));

        Logger.recordOutput("AutoHeading/TargetHeading", targetHeading.getDegrees());
        Logger.recordOutput("AutoHeading/NearestReefFace", nearestReefFace);

    }

    public Rotation2d getTargetHeading() {
        return targetHeading;
    }

    public Pose2d getNearestReefFace() {
        return nearestReefFace;
    }

    public boolean isDriverRotating(double rotationInput) {
        return Math.abs(rotationInput) > OperatorConstants.DEADBAND;
    }

    public void disableIfConflicting(Rotation2d pathPlannerTargetHeading, double toleranceDegrees) {
        double difference = Math.abs(targetHeading.minus(pathPlannerTargetHeading).getDegrees());
        // Account for wraparound (e.g., 359° vs 1° should be ~2° difference)
        if (difference > 180) {
            difference = 360 - difference;
        }
        
        if (difference > toleranceDegrees) {
            disableAutoHeading();
            Logger.recordOutput("AutoHeading/DisabledDueToConflict", true);
        } else {
            Logger.recordOutput("AutoHeading/DisabledDueToConflict", false);
        }
    }

    public void disableIfConflicting(Rotation2d pathPlannerTargetHeading) {
        disableIfConflicting(pathPlannerTargetHeading, 5.0);
    }


    @Override
    public void periodic() {
        Logger.recordOutput("AutoHeading/Enabled", autoHeadingEnabled);
    }
}
