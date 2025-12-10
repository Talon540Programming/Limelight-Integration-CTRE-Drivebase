// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drive.AutoHeading;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.ReefCentering;
import frc.robot.subsystems.Vision.VisionBase;
import frc.robot.subsystems.Vision.VisionIOLimelight;

import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.generated.TunerConstants;

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
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final VisionIOLimelight visionIO = new VisionIOLimelight();
  private final VisionBase vision = new VisionBase(visionIO, drivetrain);
  private ReefCentering reefCentering = new ReefCentering(drivetrain, vision);
  private final AutoHeading autoHeading = new AutoHeading(vision);

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond);
  private static final double MaxAngularRate = Math.PI * 2; // 1 rotation per second
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
  
  private final SwerveRequest.FieldCentricFacingAngle headingDrive = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
  
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    headingDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    headingDrive.HeadingController.setPID(7, 0, 0);

    configureBindings();
    
    drivetrain.setDefaultCommand(
      drivetrain.run(() -> {
          double xSpeed = -m_driverController.getLeftY();
          double ySpeed = -m_driverController.getLeftX();
          double rotSpeed = -m_driverController.getRightX();
          
          if (autoHeading.isEnabled() && !autoHeading.isDriverRotating(rotSpeed)) {
              autoHeading.updateTargetHeading(drivetrain.getPose());
              
              drivetrain.setControl(
                  headingDrive
                      .withVelocityX(xSpeed * MaxSpeed)
                      .withVelocityY(ySpeed * MaxSpeed)
                      .withTargetDirection(autoHeading.getTargetHeading())
              );
          } else {
              drivetrain.setControl(
                  drive
                      .withVelocityX(xSpeed * MaxSpeed)
                      .withVelocityY(ySpeed * MaxSpeed)
                      .withRotationalRate(rotSpeed * MaxAngularRate)
              );
          }
      })
    );  
    autoChooser = AutoBuilder.buildAutoChooser("default auto"); //pick a default
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  
  private void configureBindings() {
    m_driverController.povDown().onTrue(Commands.runOnce(autoHeading::toggleAutoHeading));
    m_driverController.start().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));
    m_driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));

    m_driverController.povUp().whileTrue(
        (reefCentering.createPathCommand(ReefCentering.Side.Middle).until(() -> reefCentering.haveConditionsChanged()).repeatedly()));

    m_driverController.leftBumper().whileTrue(
        (reefCentering.createPathCommand(ReefCentering.Side.Left).until(() -> reefCentering.haveConditionsChanged()).repeatedly()));

    m_driverController.rightBumper().whileTrue(
        (reefCentering.createPathCommand(ReefCentering.Side.Right).until(() -> reefCentering.haveConditionsChanged()).repeatedly()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
