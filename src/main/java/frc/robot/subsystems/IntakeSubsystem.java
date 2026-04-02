// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  private final TalonFX intakeArmMotor = new TalonFX(Constants.intakeAngleArmMotorCanID, "rio"); // change rio? will figure out specific id later for real bot
  private final TalonFX intakeWheelMotor = new TalonFX(Constants.intakeArmWheelMotorCanID, "rio");
  public IntakeSubsystem() {
    
    final PositionVoltage m_request = new PositionVoltage(Constants.intakeInAngle).withSlot(0);
    intakeArmMotor.setControl(m_request.withPosition(Constants.intakeInAngle));
  }
  public void intakeOut() {

    final PositionVoltage m_request = new PositionVoltage(Constants.intakeOutAngle).withSlot(0);
    intakeArmMotor.setControl(m_request.withPosition(Constants.intakeOutAngle));
    intakeWheelMotor.set(Constants.intakeStartSpeed);

  }

  public void  hopperOut(){

    final PositionVoltage m_request = new PositionVoltage(Constants.intakeOutAngle).withSlot(0);
    intakeArmMotor.setControl(m_request.withPosition(Constants.intakeOutAngle));
  }
  
  public void intakeIn() {
  
    final PositionVoltage m_request = new PositionVoltage(Constants.intakeInAngle).withSlot(0);
    intakeArmMotor.setControl(m_request.withPosition(Constants.intakeInAngle));
    intakeWheelMotor.set(Constants.intakeStopSpeed);
  }

  public void intakeOn() {
    intakeWheelMotor.set(Constants.intakeStartSpeed);
  }

  public void intakeOff() {
    intakeWheelMotor.set(Constants.intakeStopSpeed);
  }

  public void intakeReverse() {

    final PositionVoltage m_request = new PositionVoltage(Constants.intakeOutAngle).withSlot(0);
    intakeArmMotor.setControl(m_request.withPosition(Constants.intakeOutAngle));
    intakeWheelMotor.set(Constants.intakeReverseSpeed);
  }

  @Override
  public void periodic() {
  }
}