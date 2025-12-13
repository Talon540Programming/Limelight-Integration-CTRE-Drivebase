package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldPoses;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;



public class ReefCentering {

    private final CommandSwerveDrivetrain drivetrain;
    private VisionBase vision;
    private Pose2d nearestReefSide = new Pose2d();
    private Pose2d nearestStation = new Pose2d();

    public enum Side{
        Left,
        Middle,
        Right
    }

    public ReefCentering(CommandSwerveDrivetrain drivetrain,VisionBase vision){
        this.drivetrain = drivetrain;
        this.vision = vision;
    }
    
    public Pose2d calculateNearestReefSide(){
        if(vision.isRedAlliance()){
            return drivetrain.getPose().nearest(FieldPoses.redReefPoses);
        }
        else{
            return drivetrain.getPose().nearest(FieldPoses.blueReefPoses);
        }
    }

    public Pose2d calculateNearestStation(){
        if(vision.isRedAlliance()){
            return drivetrain.getPose().nearest(FieldPoses.redCoralStations);
        }
        else{
            return drivetrain.getPose().nearest(FieldPoses.blueCoralStations);
        }
    }

    private Pose2d calculateReefPath(Side side, Pose2d nearestSide){
        double x = nearestSide.getX();
        double y = nearestSide.getY();
        double rot = nearestSide.getRotation().getRadians();
        //tune value with more testing, rotational offset 
        rot += Math.toRadians(0);

        switch (side) {
            case Left:
                x -= FieldPoses.lateralOffset.get() * Math.sin(rot);
                y += FieldPoses.lateralOffset.get() * Math.cos(rot);
            break;
            case Right:
                x += FieldPoses.lateralOffset.get() * Math.sin(rot);
                y -= FieldPoses.lateralOffset.get() * Math.cos(rot);
            break;
            case Middle:
               default:

            break;
        }

        x += (FieldPoses.distanceOffset.get() + FieldPoses.bumperWidth) * Math.cos(rot);
        y += (FieldPoses.distanceOffset.get() + FieldPoses.bumperWidth) * Math.sin(rot);

         return new Pose2d(x, y, new Rotation2d(rot));
    }  

    private Pose2d calculateStationPath(Pose2d nearestStation){
        double x = nearestStation.getX();
        double y = nearestStation.getY();
        double rot = nearestStation.getRotation().getRadians();

        x += FieldPoses.bumperWidth * Math.cos(rot);
        y += FieldPoses.bumperWidth * Math.sin(rot);

         return new Pose2d(x, y, new Rotation2d(rot));
    }  

    public boolean haveReefConditionsChanged(){
        Pose2d nearSide = calculateNearestReefSide();
        
        return !nearSide.equals(nearestReefSide);
    }

    public boolean haveStationConditionsChanged(){
        Pose2d nearStation = calculateNearestStation();
        
        return !nearStation.equals(nearestReefSide);
    }

    private Command getPathFromWaypoint(Pose2d waypoint){
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(drivetrain.getPose().getTranslation(), getPathVelocityHeading(drivetrain.getFieldVelocity(), waypoint)), waypoint);
        
        if(waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor())< 0.01 ){
            return Commands.print("Auto alignment too close to desired position to continue");
        }

        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            Constants.PathPlannerConstants.defaultConstraints, 
            //might have to add these methods
            new IdealStartingState(drivetrain.getVelocityMagnitude(), drivetrain.getHeading()), 
            new GoalEndState(0.0, waypoint.getRotation()));
        
        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }
    //going to have to look into the drivetrain methods / make some
    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (drivetrain.getVelocityMagnitude().in(MetersPerSecond) < 0.25) {
            var diff = target.minus(drivetrain.getPose()).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
      }

    public Command createReefPathCommand(Side side){
        return Commands.defer(()-> {
            nearestReefSide = calculateNearestReefSide();
            Pose2d align = calculateReefPath(side, nearestReefSide);
            return getPathFromWaypoint(align);
        }, Set.of(drivetrain));
    }

    public Command createStationPathCommand(){
        return Commands.defer(()-> {
            nearestStation = calculateNearestStation();
            Pose2d path = calculateStationPath(nearestStation);
            return getPathFromWaypoint(path);
        }, Set.of(drivetrain));
    }
}
