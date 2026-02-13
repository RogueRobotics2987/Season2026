
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ApriltagSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretSubsystem extends SubsystemBase  {
  
  private CommandSwerveDrivetrain T_driveTrain;
  private final TalonFX turretMotor = new TalonFX(Constants.turretMotorCanID, "rio");
  private final TalonFX shooterWheelMotor = new TalonFX(Constants.shooterWheelCanID, "rio");

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(CommandSwerveDrivetrain T_driveTrain) {
    this.T_driveTrain = T_driveTrain;
    
    // The PID Controller for the turret motor
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.turretKP; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = Constants.turretKI; // no output for integrated error
    slot0Configs.kD = Constants.turretKD; // A velocity of 1 rps results in 0.1 V output

    turretMotor.getConfigurator().apply(slot0Configs);



    // Turns on continuos wrap for the turret
    var closedLoopGeneral = new ClosedLoopGeneralConfigs();
    closedLoopGeneral.ContinuousWrap = true; 
    turretMotor.getConfigurator().apply(closedLoopGeneral); 

    
    var feedback = new FeedbackConfigs();
    feedback.SensorToMechanismRatio =  Constants.turretGearRatio;
    turretMotor.getConfigurator().apply(feedback);



    var wheelSlot0Configs = new Slot0Configs();
    wheelSlot0Configs.kP = Constants.shooterWheelKP; // An error of 1 rotation results in 2.4 V output
    wheelSlot0Configs.kI = Constants.shooterWheelKI; // no output for integrated error
    wheelSlot0Configs.kD = Constants.shooterWheelKD; // A velocity of 1 rps results in 0.1 V output

    shooterWheelMotor.getConfigurator().apply(wheelSlot0Configs);


    // Applys the gear ratio to the config
    var shooterWheelFeedback = new FeedbackConfigs();
    shooterWheelFeedback.SensorToMechanismRatio =  Constants.shooterWheelGearRatio;
    shooterWheelMotor.getConfigurator().apply(shooterWheelFeedback);

  }


  public void shooter_on(){
    final VelocityVoltage m_request = new VelocityVoltage(Constants.shooterOnVelocity).withSlot(0); //leave pos blank
    shooterWheelMotor.setControl(m_request);//used to be motor
  }
  public void shooter_off(){
    final VelocityVoltage m_request = new VelocityVoltage(Constants.shooterOffVelocity).withSlot(0); //leave pos blank
    shooterWheelMotor.setControl(m_request);//used to be motor
  }

  @Override
  public void periodic() {

    // Gets Robot X, Y, Yaw
    double RobotX = T_driveTrain.getState().Pose.getX();
    double RobotY = T_driveTrain.getState().Pose.getY();
    double RobotYawRad = T_driveTrain.getState().Pose.getRotation().getRadians();

    // Calculates the global postion of the turret anywhere on the field
    double TurretXGlobal = Math.cos(RobotYawRad) * Constants.turretOffsetY + RobotX;
    double TurretYGlobal = Math.sin(RobotYawRad) * Constants.turretOffsetX + RobotY;
    SmartDashboard.putNumber("YawRad", RobotYawRad);

    // Calculates the difference in the X, Y for the Hub
    double xHubDifference = Constants.blueHubX - TurretXGlobal;
    double yHubDifference = Constants.blueHubY - TurretYGlobal;

    // Calculates the difference in the X, Y for the Passing Left
    double xPassLeftDifference = Constants.bluePassLeftX - TurretXGlobal;
    double yPassLeftDifference = Constants.bluePassLeftY - TurretXGlobal;

    // Calculates the difference in the X, Y for the Passing Right
    double xPassRightDifference = Constants.bluePassRightX - TurretXGlobal;
    double yPassRightDifference = Constants.bluePassRightY - TurretXGlobal;

    // Calculates the turret angle for the Hub in rads and outputs the numbers to SmartDashboard
    double turretAngleGlobal = Math.atan2(yHubDifference, xHubDifference) + RobotYawRad;
    SmartDashboard.putNumber("Turret Angle Hub", turretAngleGlobal);

    // Calculates the turret angle for Passing Left in rads and outputs the numbers to SmartDashboard
    double turretAnglePassLeft = Math.atan2(yPassLeftDifference, xPassLeftDifference) + RobotYawRad;
    SmartDashboard.putNumber("Turret Angle Pass Left", turretAnglePassLeft);

    // Calculates the turret angle for Passing Right in rads and outputs the numbers to SmartDashboard
    double turretAnglePassRight = Math.atan2(yPassRightDifference, xPassRightDifference) + RobotYawRad;
    SmartDashboard.putNumber("Turret Angle Pass Right", turretAnglePassRight);

    // Converts the turret angle in rads to motor rotation
    double rotations = turretAngleGlobal / (2 * Math.PI);

    // This is setting the position in rotations, so pass the converted value in.
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0); //leave pos blank
    turretMotor.setControl(m_request.withPosition(rotations));
    SmartDashboard.putNumber("Turret angle setpoint", rotations);
    SmartDashboard.putNumber("PID output", turretMotor.getClosedLoopOutput().getValueAsDouble());

  }
}
