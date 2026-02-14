// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ApriltagSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.controller.PIDController;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretSubsystem extends SubsystemBase  {

  private CommandSwerveDrivetrain T_driveTrain;
  private final TalonFX motor = new TalonFX(Constants.TurretCanID, "rio");
 
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(CommandSwerveDrivetrain T_driveTrain) {
    this.T_driveTrain = T_driveTrain;
    
    // Sets motor to brake mode
    motor.setNeutralMode(NeutralModeValue.Brake);

    // The PID Controller for the turret motor
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.turretKP; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = Constants.turretKI; // no output for integrated error
    slot0Configs.kD = Constants.turretKD; // A velocity of 1 rps results in 0.1 V output

    motor.getConfigurator().apply(slot0Configs);

    // 20 to 1 gear ratio for roomba | 15 to 1 gear ratio for robot
    double gearRatio = 20.0;

    // Turns on continuos wrap for the turret
    var closedLoopGeneral = new ClosedLoopGeneralConfigs();
    closedLoopGeneral.ContinuousWrap = true; 
    motor.getConfigurator().apply(closedLoopGeneral); 

    // Applys the gear ratio to the config
    var feedback = new FeedbackConfigs();
    feedback.SensorToMechanismRatio =  gearRatio;
    motor.getConfigurator().apply(feedback);
  }

  @Override
  public void periodic() {

    StatusSignal <Angle> motorPose = motor.getPosition();
    SmartDashboard.putNumber("Turret position", motorPose.getValueAsDouble());
    //System.out.println(motorPose.getValueAsDouble());

    Optional<Alliance> ally = DriverStation.getAlliance();

    // Gets Robot X, Y, Yaw
    double RobotX = T_driveTrain.getState().Pose.getX();
    double RobotY = T_driveTrain.getState().Pose.getY();
    double RobotYawRad = T_driveTrain.getState().Pose.getRotation().getRadians();

    // Calculates the global postion of the turret anywhere on the field
    double TurretXGlobal = Math.cos(RobotYawRad) * Constants.turretOffsetY + RobotX;
    double TurretYGlobal = Math.sin(RobotYawRad) * Constants.turretOffsetX + RobotY;
    SmartDashboard.putNumber("YawRad", RobotYawRad);

    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red){
        
        // Calculates the difference in the X, Y for the Red Hub
        double xRedHubDifference = Constants.redHubX - TurretXGlobal;
        double yRedHubDifference = Constants.redHubY - TurretYGlobal;

        // Calculates the difference in the X, Y for the Red Passing Left
        double xRedPassLeftDifference = Constants.redPassLeftX - TurretXGlobal;
        double yRedPassLeftDifference = Constants.redPassLeftY - TurretXGlobal;

        // Calculates the difference in the X, Y for the Red Passing Right
        double xRedPassRightDifference = Constants.redPassRightX - TurretXGlobal;
        double yRedPassRightDifference = Constants.redPassRightY - TurretXGlobal;

        // Calculates the turret angle for the Red Hub in rads and outputs the numbers to SmartDashboard
        double turretAngleGlobal = -(Math.atan2(yRedHubDifference, xRedHubDifference)) + RobotYawRad;
        SmartDashboard.putNumber("rad Turret Angle Red Hub", turretAngleGlobal);
        
        // Calculates the turret angle for Passing Blue Left in rads and outputs the numbers to SmartDashboard
        double turretAnglePassLeft = -(Math.atan2(yRedPassLeftDifference, xRedPassLeftDifference)) + RobotYawRad;
        SmartDashboard.putNumber("Turret Angle Red Pass Left", turretAnglePassLeft);

        // Calculates the turret angle for Passing Blue Right in rads and outputs the numbers to SmartDashboard
        double turretAnglePassRight = -(Math.atan2(yRedPassRightDifference, xRedPassRightDifference)) + RobotYawRad;
        SmartDashboard.putNumber("Turret Angle Red Pass Right", turretAnglePassRight);

        // Converts the turret angle in rads to motor rotation
        double rotations = turretAngleGlobal / (2 * Math.PI);
        SmartDashboard.putNumber("Rotations", rotations);

        // This is setting the position in rotations, so pass the converted value in.
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0); //leave pos blank
        motor.setControl(m_request.withPosition(rotations));
        SmartDashboard.putNumber("Turret angle setpoint", rotations);
        SmartDashboard.putNumber("PID output", motor.getClosedLoopOutput().getValueAsDouble());

      }

      if (ally.get() == Alliance.Blue) {

        // Calculates the difference in the X, Y for the Blue Hub
        double xHubDifference = Constants.blueHubX - TurretXGlobal;
        double yHubDifference = Constants.blueHubY - TurretYGlobal;

        // Calculates the difference in the X, Y for the Blue Passing Left
        double xBluePassLeftDifference = Constants.bluePassLeftX - TurretXGlobal;
        double yBluePassLeftDifference = Constants.bluePassLeftY - TurretXGlobal;

        // Calculates the difference in the X, Y for the Blue Passing Right
        double xBluePassRightDifference = Constants.bluePassRightX - TurretXGlobal;
        double yBluePassRightDifference = Constants.bluePassRightY - TurretXGlobal;

        // Calculates the turret angle for the Blue Hub in rads and outputs the numbers to SmartDashboard
        double turretAngleGlobal = -(Math.atan2(yHubDifference, xHubDifference)) + RobotYawRad;
        SmartDashboard.putNumber("rad Turret Angle Blue Hub", turretAngleGlobal);

        // Calculates the turret angle for Passing Blue Left in rads and outputs the numbers to SmartDashboard
        double turretAnglePassLeft = -(Math.atan2(yBluePassLeftDifference, xBluePassLeftDifference)) + RobotYawRad;
        SmartDashboard.putNumber("Turret Angle Blue Pass Left", turretAnglePassLeft);

        // Calculates the turret angle for Passing Blue Right in rads and outputs the numbers to SmartDashboard
        double turretAnglePassRight = -(Math.atan2(yBluePassRightDifference, xBluePassRightDifference)) + RobotYawRad;
        SmartDashboard.putNumber("Turret Angle Blue Pass Right", turretAnglePassRight);

        // Converts the turret angle in rads to motor rotation
        double rotations = turretAngleGlobal / (2 * Math.PI);
        SmartDashboard.putNumber("Rotations", rotations);

        // This is setting the position in rotations, so pass the converted value in.
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0); //leave pos blank
        motor.setControl(m_request.withPosition(rotations));
        SmartDashboard.putNumber("Turret angle setpoint", rotations);
        SmartDashboard.putNumber("PID output", motor.getClosedLoopOutput().getValueAsDouble());

      }
    }
  }
}