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
    
    // try{
    //   Thread.sleep(3000);
    // }catch(InterruptedException e){}
    // var slot0Configs = new Slot0Configs();
    // slot0Configs.kP = Constants.intake_kP; // An error of 1 rotation results in 2.4 V output
    // slot0Configs.kI = Constants.intake_kI; // no output for integrated error
    // slot0Configs.kD = Constants.intake_kD; // A velocity of 1 rps results in 0.1 V output
    // intakeArmMotor.getConfigurator().apply(slot0Configs);
    // System.out.println("hi");
    
    // var fx_cfg = new TalonFXConfiguration();
    // fx_cfg.Feedback.FeedbackRemoteSensorID = Constants.intakeAngleArmCanCoderCanID;
    // fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // intakeArmMotor.getConfigurator().apply(fx_cfg);

    // var feedback = new FeedbackConfigs();
    // feedback.SensorToMechanismRatio =  Constants.intakeArmGearRatio;
    // intakeArmMotor.getConfigurator().apply(feedback);
  }
  public void intakeOut() {

    final PositionVoltage m_request = new PositionVoltage(Constants.intakeOutAngle).withSlot(0);
    //here
    intakeArmMotor.setControl(m_request.withPosition(Constants.intakeOutAngle));
    intakeWheelMotor.set(Constants.intakeStartSpeed);

  }
  
  public void intakeIn() {
  
    final PositionVoltage m_request = new PositionVoltage(Constants.intakeInAngle).withSlot(0);
    //here 
    intakeArmMotor.setControl(m_request.withPosition(Constants.intakeInAngle));
    intakeWheelMotor.set(Constants.intakeStopSpeed);
  }

  public void intakeOn() {
    intakeWheelMotor.set(Constants.intakeStartSpeed);
  }

  public void intakeOff() {
    intakeWheelMotor.set(Constants.intakeStopSpeed);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
