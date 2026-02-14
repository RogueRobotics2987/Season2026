// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {
  // <type> <name> = <value>;


  // <type> <name> = <value>;

  public final TalonFX motorkicker11 = new TalonFX(Constants.KickerCanID, "rio"); //change rio?

  public final TalonFX motorShooterWheels = new TalonFX(Constants.ShooterWheelsCanID, "rio"); //change rio?


  /** Creates a new KickerSubSystem. */
  public KickerSubsystem() { 


    System.out.println ("uyuioiytriytr");
    // Setting Up PID Controller;
    var slot0Configs = new Slot0Configs(); // Creating a new object (instantiation)
    slot0Configs.kP = 0.1; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

    // ============ 
    
    // equivalent 
    motorkicker11.getConfigurator().apply(slot0Configs);
    motorShooterWheels.getConfigurator().apply(slot0Configs);
    // ============

   
    var feedback = new FeedbackConfigs();
    feedback.SensorToMechanismRatio =  Constants.Kicker16;

    // ============
    // split into 2 lines of code here
   
    // equivalent
    motorkicker11.getConfigurator().apply(feedback);
    // ===========
  }

  // Add two public void functions START and STOP sets velocity START sets veloctity to a 
  //certain speed and STOP sets the velocity to zero
  public void Start() { 
    final VelocityVoltage m_request = new VelocityVoltage(Constants.kickerOnspeed).withSlot(0);
    final VelocityVoltage m_shooter = new VelocityVoltage(Constants.shooterOnSpeed).withSlot(0);
    motorkicker11.setControl(m_request);
    motorShooterWheels.setControl(m_shooter);
  };

  public void Stop() {
    final VelocityVoltage m_request = new VelocityVoltage(Constants.kickerOffspeed).withSlot(0);
    final VelocityVoltage m_shooter = new VelocityVoltage(Constants.shooterOffSpeed).withSlot(0);
    motorkicker11.setControl(m_request);
    motorShooterWheels.setControl(m_shooter);
  };

   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}