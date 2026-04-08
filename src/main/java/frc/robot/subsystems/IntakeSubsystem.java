// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Do we need some of these Imports? (The ones that aren't used)
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

 /** Creates a new IntakeSubsystem. */
public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeArmMotor = new TalonFX(Constants.intakeAngleArmMotorCanID, "rio");
  private final TalonFX intakeWheelMotor = new TalonFX(Constants.intakeArmWheelMotorCanID, "rio");
  
  public IntakeSubsystem() {}
  public void intakeOut() {
    // intakeWheelMotor.set(Constants.intakeStartSpeed);
    final PositionVoltage m_request = new PositionVoltage(Constants.intakeOutAngle).withSlot(0);
    intakeArmMotor.setControl(m_request.withPosition(Constants.intakeOutAngle));
  }

  public void intakeIn() {
    // intakeWheelMotor.set(Constants.intakeStopSpeed);
    final PositionVoltage m_request = new PositionVoltage(Constants.intakeInAngle).withSlot(0);
    intakeArmMotor.setControl(m_request.withPosition(Constants.intakeInAngle));
  }

  // TODO: Add delays between intake arm up and down.
  public void intakeFeed() {
    intakeWheelMotor.set(Constants.intakeStartSpeed);
    PositionVoltage m_request = new PositionVoltage(Constants.intakeFeedAngle).withSlot(0);
    intakeArmMotor.setControl(m_request);
    // delay here
    m_request = new PositionVoltage(Constants.intakeOutAngle).withSlot(0);
    intakeArmMotor.setControl(m_request);
    // delay here
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

  public Command getIntakeOutCommand(){
    return this.runOnce(() -> { intakeOut(); intakeOn(); });
  }

  public Command getIntakeInCommand(){
    return this.runOnce(() -> { intakeOff(); intakeIn(); });
  }

  @Override
  public void periodic() {}
}