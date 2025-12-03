// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Vision.ReefCentering;
import frc.robot.subsystems.Vision.VisionBase;
import frc.robot.subsystems.Vision.VisionIOLimelight;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain();
  private final VisionIOLimelight visionIO = new VisionIOLimelight();
  private final VisionBase vision = new VisionBase(visionIO, drivetrain);
  private ReefCentering reefCentering = new ReefCentering(drivetrain, visionIO);

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser("default auto"); //pick a default
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  
  private void configureBindings() {
    Commands.runOnce(drivetrain::seedFieldCentric);
    m_driverController.povUp().whileTrue(reefCentering.createPathCommand(ReefCentering.Side.Middle).until(() -> reefCentering.haveConditionsChanged()).repeatedly());
    m_driverController.leftBumper().whileTrue(reefCentering.createPathCommand(ReefCentering.Side.Left).until(() -> reefCentering.haveConditionsChanged()).repeatedly());
    m_driverController.rightBumper().whileTrue(reefCentering.createPathCommand(ReefCentering.Side.Right).until(() -> reefCentering.haveConditionsChanged()).repeatedly());

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
