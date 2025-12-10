// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;
  }

  public static final String kLimelightName = "limelight";

  public static enum Reef {
    left,
    right,
    forward;
  }


  public static final class PathPlannerConstants {

    public static final PathConstraints testingConstraints = new PathConstraints(
        Units.feetToMeters(1.5), 2.0,             
        Units.degreesToRadians(50), Units.degreesToRadians(300));

    public static final PathConstraints slowConstraints = new PathConstraints(
        Units.feetToMeters(3.5), 4.0,             
        Units.degreesToRadians(100), Units.degreesToRadians(720));

    public static final PathConstraints defaultConstraints = new PathConstraints(
        Units.feetToMeters(8), 4.0,
        Units.degreesToRadians(200), Units.degreesToRadians(720));

    public static final PathConstraints fastConstraints = new PathConstraints(
      Units.feetToMeters(14), 4.0,
        Units.degreesToRadians(360), Units.degreesToRadians(720));
  }

  public static final class FieldPoses {

    public static final double[] fieldSize = {17.55, 8.05};
    // Wall thickness is 0.051
    public static final Pose2d blueCenterOfReef = new Pose2d(4.487, 4.025, new Rotation2d()); // blue
    public static final Pose2d redCenterOfReef = new Pose2d(13.065, 4.025, new Rotation2d()); // red

    public static final List<Pose2d> blueReefPoses = new ArrayList<Pose2d>(){{
      add(new Pose2d(2.890, 4.025, new Rotation2d(Units.degreesToRadians(0))));
      add(new Pose2d(3.689, 2.642, new Rotation2d(Units.degreesToRadians(60.0))));
      add(new Pose2d(5.285, 2.642, new Rotation2d(Units.degreesToRadians(120.0))));
      add(new Pose2d(6.087, 4.025, new Rotation2d(Units.degreesToRadians(180.0))));
      add(new Pose2d(5.285, 5.408, new Rotation2d(Units.degreesToRadians(240.0))));
      add(new Pose2d(3.689, 5.408, new Rotation2d(Units.degreesToRadians(300.0))));
    }};

    public static final List<Pose2d> redReefPoses = new ArrayList<Pose2d>(){{
      add(new Pose2d(11.466, 4.025, new Rotation2d(Units.degreesToRadians(0))));
      add(new Pose2d(12.265, 2.642, new Rotation2d(Units.degreesToRadians(60.0))));
      add(new Pose2d(13.861, 2.642, new Rotation2d(Units.degreesToRadians(120.0))));
      add(new Pose2d(14.663, 4.025, new Rotation2d(Units.degreesToRadians(180.0))));
      add(new Pose2d(13.861, 5.408, new Rotation2d(Units.degreesToRadians(240.0))));
      add(new Pose2d(12.265, 5.408, new Rotation2d(Units.degreesToRadians(300.0))));
    }};
    
    public static final double Offset = 0.165;
  }
}
