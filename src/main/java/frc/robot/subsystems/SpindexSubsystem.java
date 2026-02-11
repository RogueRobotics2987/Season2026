// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of thisp project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.robot.Constants;

public class SpindexSubsystem extends SubsystemBase {

  private final TalonFX SpindexMotor = new TalonFX(20, "rio"); //change rio?
  /** Creates a new spindexSubsystem. */
  public SpindexSubsystem() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.kP; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = Constants.kI; // no output for integrated error
    slot0Configs.kD = Constants.kD; // A velocity of 1 rps results in 0.1 V output

    // Applys the gear ratio to the config
    var feedback = new FeedbackConfigs();
    feedback.SensorToMechanismRatio =  Constants.spindexGearRatio;
    SpindexMotor.getConfigurator().apply(feedback);
  

    SpindexMotor.getConfigurator().apply(slot0Configs);
    System.out.println("yo");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final VelocityVoltage m_request = new VelocityVoltage(100).withSlot(0); //leave pos blank
    SpindexMotor.setControl(m_request);
  }

  public void start(){
   final VelocityVoltage m_request = new VelocityVoltage(Constants.spindexOnSpeed).withSlot(0);
   SpindexMotor.setControl(m_request);
  }
  public void stop(){
   final VelocityVoltage m_request = new VelocityVoltage(Constants.spindexOffSpeed).withSlot(0);
   SpindexMotor.setControl(m_request);
  }
}

