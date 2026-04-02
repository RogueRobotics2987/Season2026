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

public class IndexSubsystem extends SubsystemBase {

  private final TalonFX IndexMotor = new TalonFX(Constants.indexMotorCanID, "rio"); // 
  /** Creates a new indexSubsystem. */
  public IndexSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void start(){
   final VelocityVoltage m_request = new VelocityVoltage(Constants.indexOnSpeed).withSlot(0);
   IndexMotor.setControl(m_request.withVelocity(Constants.indexOnSpeed));
  }
  public void stop(){
   final VelocityVoltage m_request = new VelocityVoltage(Constants.indexOffSpeed).withSlot(0);
   IndexMotor.setControl(m_request.withVelocity(Constants.indexOffSpeed));
  }
}

