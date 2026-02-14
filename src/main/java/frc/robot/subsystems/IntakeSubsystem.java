// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  private final TalonFX intakeMotor = new TalonFX(Constants.IntakeCanID, "rio"); // change rio? will figure out specific id later for real bot
  
  public IntakeSubsystem() {
    
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.intake_kP; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = Constants.intake_kI; // no output for integrated error
    slot0Configs.kD = Constants.intake_kD; // A velocity of 1 rps results in 0.1 V output
    
    intakeMotor.getConfigurator().apply(slot0Configs);
    System.out.println("hi");
  }
  
  
  @Override
  public void periodic() {
    var fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackRemoteSensorID = intakeMotor.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    
    intakeMotor.getConfigurator().apply(fx_cfg);
    // This method will be called once per scheduler run
  }
}
