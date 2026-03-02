// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {
  /** Creates a new LightSubsystem. */

//this code is created and copyrighted by bRANDON AND NOBODY ELSE

//brandon says this looks okie doki

// create 3 digital outputs
// make 5 functions for changing light state
 
 // pin setups
  DigitalOutput output1 = new DigitalOutput(4);   // ids subject to change. put ids in constants?
 
  DigitalOutput output2 = new DigitalOutput(5);
 
  DigitalOutput output3 = new DigitalOutput(6);

  public LightSubsystem() {
    
  }
  

  //set of functions for changing lights
   public void Off() {
    //boolean vars
    output1.set(false);
    output2.set(false);
    output3.set(false); 
   }

  public void Greenblink() {
    output1.set(true);
    output2.set(false);
    output3.set(false);
  }

  public void Red() {
    output1.set(false);
    output2.set(true);
    output3.set(false);
  }

  public void Purple() {
    output1.set(false);
    output2.set(false);
    output3.set(true);
  }

  public void Purpleblink() {
    output1.set(true);
    output2.set(true);
    output3.set(false);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //for testing the outputs on the smart dashboard:

    //SmartDashboard.putBoolean("output1", output1.get());
    //SmartDashboard.putBoolean("output2", output2.get());
    //SmartDashboard.putBoolean("output3", output3.get());
  }
}
