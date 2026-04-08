// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of thisp project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  // SmartDashboard.putBoolean("Index", true);
   final VelocityVoltage m_request = new VelocityVoltage(Constants.indexOnSpeed).withSlot(0);
   IndexMotor.setControl(m_request.withVelocity(Constants.indexOnSpeed));
  }

  public void startBackward(){
   final VelocityVoltage m_request = new VelocityVoltage(Constants.indexReverseSpeed).withSlot(0);
   IndexMotor.setControl(m_request.withVelocity(Constants.indexReverseSpeed));
  }

  public void stop(){
    // SmartDashboard.putBoolean("Index", false);
   final VelocityVoltage m_request = new VelocityVoltage(Constants.indexOffSpeed).withSlot(0);
   IndexMotor.setControl(m_request.withVelocity(Constants.indexOffSpeed));
  }

   public Command getStartBackwardsCommand(){
    return this.runOnce(() -> { startBackward(); });
  }

}



// Replace 'm_armMotor' with your TalonFX object name
//var motorState = m_armMotor.getStickyFaultField().getValue();
//var controlStatus = m_armMotor.getAppliedControl().toString();

// 1. Check for Hardware Limit Switches (even if not physically there)
//boolean forwardLimit = m_armMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
//boolean reverseLimit = m_armMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;

// 2. Check for Software Limits
//boolean forwardSoftLimit = m_armMotor.getFault_ForwardSoftLimit().getValue();
//boolean reverseSoftLimit = m_armMotor.getFault_ReverseSoftLimit().getValue();

// 3. Log to SmartDashboard for easy viewing
//SmartDashboard.putBoolean("Arm/Forward Limit Trig", forwardLimit);
//SmartDashboard.putBoolean("Arm/Reverse Limit Trig", reverseLimit);
//SmartDashboard.putBoolean("Arm/Forward Soft Limit Trig", forwardSoftLimit);
//SmartDashboard.putBoolean("Arm/Reverse Soft Limit Trig", reverseSoftLimit);
//SmartDashboard.putNumber("Arm/Current Position", m_armMotor.getPosition().getValueAsDouble());

// 4. Print if a limit is actively killing power (The Red Blink)
//if (forwardLimit || reverseLimit || forwardSoftLimit || reverseSoftLimit) {
//    System.out.println("KRAKEN HALTED: Limit Triggered! Check Tuner X Configs.");
//}

