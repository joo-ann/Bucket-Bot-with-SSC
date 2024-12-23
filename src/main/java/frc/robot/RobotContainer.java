// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoSpinUpShoot;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// This class is where the bulk of the robot should be declared.  Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the Robot
// periodic methods (other than the scheduler calls).  Instead, the structure of the robot
// (including subsystems, commands, and button mappings) should be declared here.
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final LEDSSubsystem m_ledsSubsystem = new LEDSSubsystem();

  // Create New Choosing Option in SmartDashboard for Autos
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.k_driverController);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    // Makes the drive command the default command (good!)
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

    // Named Command Configuration
    NamedCommands.registerCommand("Rev Up Shoot", new AutoSpinUpShoot(m_shooterSubsystem, m_indexerSubsystem));

    // Autos
    m_chooser.addOption("Reverse", m_swerveSubsystem.getAutonomousCommand("Reverse"));
    m_chooser.addOption("Shoot Only", m_swerveSubsystem.getAutonomousCommand("Shoot"));
    m_chooser.addOption("Drive Across Line", m_swerveSubsystem.getAutonomousCommand("Drive Across Line"));
    m_chooser.addOption("Drive Forward & Shoot", m_swerveSubsystem.getAutonomousCommand("Drive Forward & Shoot"));
    m_chooser.addOption("Drive Forward, Shoot, Drive Back", m_swerveSubsystem.getAutonomousCommand("Drive Forward, Shoot, Drive Back"));
    m_chooser.addOption("Drive Forward, Shoot, Move Back", m_swerveSubsystem.getAutonomousCommand("Drive Forward, Shoot, Move Back"));

    // Puts a chooser on the SmartDashboard!
    SmartDashboard.putData("AutoMode", m_chooser);
  }

  // Trigger & Button Bindings!
  private void configureBindings() {

    // Spin Up - Right Trig
    new Trigger(() -> m_driverController.getRawAxis(DriveConstants.k_spinUpTrigger) > 0.05)

      .whileTrue(
        new InstantCommand(() -> m_shooterSubsystem.spinUp(), m_shooterSubsystem).alongWith(
        new InstantCommand(() -> m_indexerSubsystem.run(), m_indexerSubsystem)).alongWith(
        new InstantCommand(() -> m_ledsSubsystem.setViolet(), m_ledsSubsystem))
      )
      .whileFalse(
        new InstantCommand(() -> m_shooterSubsystem.stop(), m_shooterSubsystem).alongWith(
        new InstantCommand(() -> m_indexerSubsystem.stop(), m_indexerSubsystem)).alongWith(
        new InstantCommand(() -> m_ledsSubsystem.setTeamColor(), m_ledsSubsystem))
      );
    
    // Pew pew! - Left Trig
    new Trigger(() -> m_driverController.getRawAxis(DriveConstants.k_shootTrigger) > 0.05)
      .whileTrue(
        new InstantCommand(() -> m_shooterSubsystem.shoot(), m_shooterSubsystem).alongWith(
        new InstantCommand(() -> m_indexerSubsystem.run(), m_indexerSubsystem))
      )
      .whileFalse(
        new InstantCommand(() -> m_shooterSubsystem.stopShoot(), m_shooterSubsystem).alongWith(
        new InstantCommand(() -> m_indexerSubsystem.stop(), m_indexerSubsystem))
      );

    // Zero Gyro - Start Button
    new JoystickButton(m_driverController.getHID(), DriveConstants.k_zeroGyroButton)
      .whileTrue(
        new InstantCommand(() -> m_swerveSubsystem.zeroGyro(), m_swerveSubsystem).alongWith(
        new InstantCommand(() -> m_ledsSubsystem.setHotPink()))
      );

    // Test Auto Shoot Stuff
    // new JoystickButton(m_driverController.getHID(), DriveConstants.k_testButton)
    //   .whileTrue(
    //     new AutoSpinUpShoot(m_shooterSubystem, m_indexerSubsystem)
    //   );
  }
  
  // Command that takes Xbox Controller Inputs and allows robot to drive
  // NOTE: getLeftY and getLeftX are opposite for a reason!!! It is correct!!
  Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveCommand(
      () -> MathUtil.applyDeadband(m_driverController.getLeftY() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
      () -> MathUtil.applyDeadband(m_driverController.getLeftX() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
      () -> m_driverController.getRightX() * DriveConstants.k_turnRate);

  // Use this to pass the autonomous command to Robot.java
  // Returns the command to run in autonomous
  public Command getAutonomousCommand() {

    // The selected auto on SmartDashboard will be run in autonomous
    return m_chooser.getSelected(); 
  }
}
