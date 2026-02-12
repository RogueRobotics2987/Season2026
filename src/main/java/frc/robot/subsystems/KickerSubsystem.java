// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {
  private final TalonFX motorkicker11 = new TalonFX(11, "rio"); //change rio?
  /** Creates a new KickerSubSystem. */
  public KickerSubsystem() { 


    System.out.println ("uyuioiytriytr");
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.kickerKP; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = Constants.kickerKI; // no output for integrated error
    slot0Configs.kD = Constants.kickerKD; // A velocity of 1 rps results in 0.1 V output

    motorkicker11.getConfigurator().apply(slot0Configs);

    var feedback = new FeedbackConfigs();
    feedback.SensorToMechanismRatio =  1;
    motorkicker11.getConfigurator().apply(feedback);
  }

  // Add two public void functions START and STOP sets velocity START sets veloctity to a 
  //certain speed and STOP sets the velocity to zero
  public void Start() { 
    final VelocityVoltage m_request = new VelocityVoltage(100).withSlot(0);
    motorkicker11.setControl(m_request);
  };

  public void Stop() {
   final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
   motorkicker11.setControl(m_request);
  };

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    motorkicker11.setControl(m_request.withVelocity(100));
  }
}
