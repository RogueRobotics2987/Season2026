// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase  {

  private CommandSwerveDrivetrain T_driveTrain; // corey said make constants B^(
  private final TalonFX motorTurret = new TalonFX(Constants.turretMotorID, "rio");
  private final TalonFX motorKicker = new TalonFX(Constants.kickerMotorID, "rio");
  private final TalonFX motorShooterWheels = new TalonFX(Constants.shooterWheelMotorID, "rio");

  public static enum AimTarget {
    AUTO,
    LEFT,
    RIGHT
  }
 
  private AimTarget Target = AimTarget.AUTO;

  /** Creates a new TurretSubsystem. */
  public ShooterSubsystem(CommandSwerveDrivetrain T_driveTrain) {
    this.T_driveTrain = T_driveTrain;
    
    // Sets motor to brake mode
    motorTurret.setNeutralMode(NeutralModeValue.Brake);

    // The PID Controller for the turret motor
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.turretKP; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = Constants.turretKI; // no output for integrated error
    slot0Configs.kD = Constants.turretKD; // A velocity of 1 rps results in 0.1 V output

    // the PID Controller for the kicker motor
    Slot0Configs kickerSlot0Configs = new Slot0Configs();
    kickerSlot0Configs.kP = Constants.kickerKP; // An error of 1 rotation results in 2.4 V output
    kickerSlot0Configs.kI = Constants.kickerKI; // no output for integrated error
    kickerSlot0Configs.kD = Constants.kickerKD; // A velocity of 1 rps results in 0.1 V output

    // the PID Controller for the shooter motor
    Slot0Configs shooterWheelsSlot0Configs = new Slot0Configs();
    shooterWheelsSlot0Configs.kP = Constants.shooterWheelsKP; // An error of 1 rotation results in 2.4 V output
    shooterWheelsSlot0Configs.kI = Constants.shooterWheelsKP; // no output for integrated error
    shooterWheelsSlot0Configs.kD = Constants.shooterWheelsKP; // A velocity of 1 rps results in 0.1 V output

    motorTurret.getConfigurator().apply(slot0Configs);
    motorKicker.getConfigurator().apply(kickerSlot0Configs);
    motorShooterWheels.getConfigurator().apply(shooterWheelsSlot0Configs);

    // Turns on continuos wrap for the turret
    ClosedLoopGeneralConfigs closedLoopGeneral = new ClosedLoopGeneralConfigs();
    closedLoopGeneral.ContinuousWrap = true; 
    motorTurret.getConfigurator().apply(closedLoopGeneral); 

    // Applys the gear ratio to the config
    FeedbackConfigs feedback = new FeedbackConfigs();
    feedback.SensorToMechanismRatio = Constants.turretGearRatio;
    motorTurret.getConfigurator().apply(feedback);
  }

  public void SetTarget(AimTarget NewTarget) {
    Target = NewTarget;
  }

  public void StartREV() { // JEFF DOESNT LIKE THE NAME
    final VelocityVoltage m_kickerRequest = new VelocityVoltage(Constants.kickerOnspeed).withSlot(0); // COREY SAID COULD BE MEMBER VARIBLES
    final VelocityVoltage m_shooterRequest = new VelocityVoltage(Constants.shooterOnSpeed).withSlot(0);
    motorKicker.setControl(m_kickerRequest.withVelocity(Constants.kickerOnspeed));
    motorShooterWheels.setControl(m_shooterRequest.withVelocity(Constants.shooterOnSpeed));
  }

  public void StopREV() {
    final VelocityVoltage m_kickerRequest = new VelocityVoltage(Constants.kickerOffspeed).withSlot(0);
    final VelocityVoltage m_shooterRequest = new VelocityVoltage(Constants.shooterOffSpeed).withSlot(0);
    motorKicker.setControl(m_kickerRequest.withVelocity(Constants.kickerOffspeed));
    motorShooterWheels.setControl(m_shooterRequest.withVelocity(Constants.shooterOffSpeed));
  }

  @Override
  public void periodic() {

    StatusSignal <Angle> motorPose = motorTurret.getPosition();
    SmartDashboard.putNumber("Turret position", motorPose.getValueAsDouble());
    //System.out.println(motorPose.getValueAsDouble()); // JeFf DoEsNt LiKe ThIs CoMmEnT // jEfF dOeSnT lIkE tHiS cOmMeNt // if you couldnt tell brodie was here

    Optional<Alliance> ally = DriverStation.getAlliance();

    // Gets Robot X, Y, Yaw
    double RobotX = T_driveTrain.getState().Pose.getX();
    double RobotY = T_driveTrain.getState().Pose.getY();
    double RobotYawRad = T_driveTrain.getState().Pose.getRotation().getRadians();

    // Calculates the global postion of the turret anywhere on the field
    double TurretXGlobal = Math.cos(RobotYawRad) * Constants.turretOffsetY + RobotX;
    double TurretYGlobal = Math.sin(RobotYawRad) * Constants.turretOffsetX + RobotY;
    SmartDashboard.putNumber("YawRad", RobotYawRad);

    double targetX = 1;
    double targetY = 0;

    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red){
        targetX = Constants.redHubX;
        targetY = Constants.redHubY;

        if (TurretXGlobal < Constants.redHubX && TurretYGlobal < Constants.redHubY || Target == AimTarget.LEFT) {
          targetX = Constants.redPassLeftX;
          targetY = Constants.redPassLeftY;
        }

        if (TurretXGlobal < Constants.redHubX && TurretYGlobal > Constants.redHubY || Target == AimTarget.RIGHT) {
          targetX = Constants.redPassRightX;
          targetY = Constants.redPassRightY;
        }
      }

      if (ally.get() == Alliance.Blue) {
        targetX = Constants.blueHubX;
        targetY = Constants.blueHubY;

        if (TurretXGlobal < Constants.blueHubX && TurretYGlobal < Constants.blueHubY || Target == AimTarget.RIGHT) {
          targetX = Constants.bluePassRightX;
          targetY = Constants.bluePassRightY;
        }

        if (TurretXGlobal < Constants.blueHubX && TurretYGlobal > Constants.blueHubY || Target == AimTarget.LEFT) {
          targetX = Constants.bluePassLeftX;
          targetY = Constants.bluePassLeftY;
        }
      }
    }

    // Calculates the difference in the X, Y for the target
    double xDifference = targetX - TurretXGlobal;
    double yDifference = targetY - TurretYGlobal;

    // Calculates the turret angle for the target in rads
    double turretAngleGlobal = -(Math.atan2(yDifference, xDifference)) + RobotYawRad;
    SmartDashboard.putNumber("rad Turret Angle Red Hub", turretAngleGlobal);

    // Converts the turret angle in rads to motor rotation
    double rotations = turretAngleGlobal / (2 * Math.PI);
    SmartDashboard.putNumber("Rotations", rotations);

    // This is setting the position in rotations, so pass the converted value in.
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0); //leave pos blank
    motorTurret.setControl(m_request.withPosition(rotations));
    SmartDashboard.putNumber("Turret angle setpoint", rotations);
    SmartDashboard.putNumber("PID output", motorTurret.getClosedLoopOutput().getValueAsDouble());

  }
}