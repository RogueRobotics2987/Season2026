// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Do we need some of these Imports? (The ones that aren't used)
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.concurrent.DelayQueue;
import java.util.concurrent.Delayed;

import org.opencv.core.Mat;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

 /** Creates a new IntakeSubsystem. */
public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeArmMotor = new TalonFX(Constants.intakeAngleArmMotorCanID, "rio");
  private final TalonFX intakeWheelMotor = new TalonFX(Constants.intakeArmWheelMotorCanID, "rio");

  private double armCurrent = 0.0;
  private int overCurrentCount = 0;
  private boolean armStalled = false;
  private double intakeArmTargetPosition = 0.0;
  private double currentPosition = 0.0;

  
  public IntakeSubsystem() {}
  public void intakeOut() {
    intakeWheelMotor.set(Constants.intakeStartSpeed);
    intakeArmTargetPosition = Constants.intakeOutAngle;
    final PositionVoltage m_request = new PositionVoltage(Constants.intakeOutAngle).withSlot(0);
    intakeArmMotor.setControl(m_request.withPosition(Constants.intakeOutAngle));
  }

  public void intakeIn() {
    intakeWheelMotor.set(Constants.intakeStopSpeed);
    intakeArmTargetPosition = Constants.intakeInAngle;
    final PositionVoltage m_request = new PositionVoltage(Constants.intakeInAngle).withSlot(0);
    intakeArmMotor.setControl(m_request.withPosition(Constants.intakeInAngle));
  }

  // TODO: Add delays between intake arm up and down.
  public void intakeFeed() {
    intakeWheelMotor.set(Constants.intakeStartSpeed);
    intakeArmTargetPosition = Constants.intakeFeedAngle;
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
  
  public boolean isArmStalled(){
    return armStalled;
  }

  public void clearStall(){
    armStalled = false;
    overCurrentCount = 0;
  }

  public boolean isIntakeArmAtTarget(){
    currentPosition = intakeArmMotor.getPosition().getValueAsDouble();
    return Math.abs(currentPosition - intakeArmTargetPosition) < Constants.intakeArmPositionTolerance;
  }

  public Command getIntakeOutCommand(){
    return this.runOnce(() -> { clearStall(); intakeOut(); })
    .until(() -> isIntakeArmAtTarget() || isArmStalled());
  }

  public Command getIntakeInCommand(){
    return this.runOnce(() -> { clearStall(); intakeIn(); })
    .until(() -> isIntakeArmAtTarget() || isArmStalled());
  }

  @Override
  public void periodic() {
    armCurrent = intakeArmMotor.getStatorCurrent().getValueAsDouble();

    if (armCurrent > Constants.armCurrentThreshold) {
      overCurrentCount++;
      if (overCurrentCount >= Constants.armCurrentThreshold) {
        intakeArmMotor.set(0); // Kill the motor immediately
        armStalled = true;
      }
    } else {
      overCurrentCount = 0;
    }
  }
}