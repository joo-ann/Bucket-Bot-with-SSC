// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.MotorIDConstants;

// Indexer Subsystem Code yippee
public class IndexerSubsystem extends SubsystemBase {

  // Init Stuff
  private CANSparkMax m_indexerNEO;

  // Other setup items
  public IndexerSubsystem() {
    
    // Set up NEO 550
    m_indexerNEO = new CANSparkMax(MotorIDConstants.k_indexerNEOID, MotorType.kBrushless);

    // RAMP RATE IS SO IMPORTANT, tells motor how fast it can speed up
    m_indexerNEO.setOpenLoopRampRate(0.25);
  }

  // Runs once per scheduler run
  public void periodic() {}
  
  // Indexes yippeee
  public void run() {
    m_indexerNEO.set(IndexerConstants.k_indexerNEOSpeed);
  }

  // Stops motors
  public void stop() {
    m_indexerNEO.set(0);
  }
}
