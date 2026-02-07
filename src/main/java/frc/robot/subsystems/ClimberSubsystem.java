// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMoter = new TalonFX(20, "rio");
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
     var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 20; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

    climberMoter.getConfigurator().apply(slot0Configs);

    var feedback = new FeedbackConfigs();
    feedback.SensorToMechanismRatio =  Constants.climberNumber;
    climberMoter.getConfigurator().apply(feedback);

    System.out.println ("Guess what? Hello world!");
  
  }

  // add new functions to your class here
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0); //leave pos blank
    climberMoter.setControl(m_request.withPosition(100)); 
  }
  
  //make public void function that is climb to peramiters will set desired posion to var that will connect to a magic num, sec public unclimb set back to diffrent var then connets to magic
 
  public void climberUp(){
    final PositionVoltage m_request = new PositionVoltage(20).withSlot(0);
    climberMoter.setControl(m_request.withPosition(100));
    }

   public void climberDown(){
    final PositionVoltage m_request = new PositionVoltage(20).withSlot(0);
    climberMoter.setControl(m_request.withPosition(0));
    }
}
