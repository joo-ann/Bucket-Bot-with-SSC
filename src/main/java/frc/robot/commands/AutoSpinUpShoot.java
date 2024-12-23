// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSpinUpShoot extends Command {

  // Instantiate Stuff
  ShooterSubsystem m_ShooterSubsystem;
  IndexerSubsystem m_IndexerSubsystem;
  Timer timer = new Timer();

  public AutoSpinUpShoot(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_ShooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
    m_IndexerSubsystem = indexerSubsystem;
    addRequirements(indexerSubsystem);
  }

  // Reset timer when the command starts executing
  public void initialize() {
    timer.start();
    timer.reset();
  }
  
  // Actual command
  public void execute() {

    // Spin up and run indexer for the first spinUpAutoTime seconds
    if(timer.get() < AutoConstants.spinUpAutoTime){
      m_ShooterSubsystem.spinUp();
      m_IndexerSubsystem.run();
    }

    // Shoot and run indexer from the shootAutoTime to the spinUpAutoTime
    if(timer.get() > AutoConstants.shootAutoTime && timer.get() < AutoConstants.spinUpAutoTime){
      m_ShooterSubsystem.shoot();
      m_IndexerSubsystem.run();
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
    m_ShooterSubsystem.stop();
    m_IndexerSubsystem.stop();
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return timer.get() > AutoConstants.spinUpAutoTime;
  }
}
