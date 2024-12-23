// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;

public class LEDSSubsystem extends SubsystemBase {

    // Init Stuff
    Spark blinkin;

  public LEDSSubsystem() {

    // Set up blinkin
    blinkin = new Spark(MotorIDConstants.blinkinPort);
  }

  @Override
  public void periodic() {
    
    // Set team color
    setTeamColor();

  }

  public void setRed(){
    blinkin.set(0.61);
  }

  public void setBlue(){
    blinkin.set(0.87);
  }

  public void setGreen(){
    blinkin.set(0.77);
  }

  public void setYellow(){
    blinkin.set(0.69);
  }

  public void setViolet(){
    blinkin.set(0.91);
  }

  public void setRainbowPrettyyy(){
    blinkin.set(-0.99);
  }

  public void setTwinklesParty(){
    blinkin.set(-0.53);
  }

  public void setStrobe(){
    blinkin.set(0.15);
  }

  public void setFire(){
    blinkin.set(-0.59);
  }

  public void setHotPink(){
    blinkin.set(0.57);
  }

  public void turnOff(){
    blinkin.set(0);
  }

  public void setTwinklesLavaParty(){
    blinkin.set(-0.49);
  }

  public void setTwinklesOceanParty(){
    blinkin.set(-0.51);
  }

  public void setTeamColor(){
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Red) {
        setRed();
      }
      else if(alliance.get() == DriverStation.Alliance.Blue) {
        setBlue();
      }
      else {
        setGreen();
      }
    }
  }

   public void setTeamColorTwinkles(){
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Red) {
          setTwinklesLavaParty();
        }
      else if(alliance.get() == DriverStation.Alliance.Blue){
          setTwinklesOceanParty();
        }
    }
  }
}
