// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final TalonFX intakeMotor = new TalonFX(9, "rio"); // change rio? will figure out specific id later for real bot
  
  public IntakeSubsystem() {

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.1; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    intakeMotor.getConfigurator().apply(slot0Configs);
   System.out.println("hi");
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
