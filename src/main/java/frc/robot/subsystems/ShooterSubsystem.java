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
import java.lang.Math;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase  {

  private CommandSwerveDrivetrain T_driveTrain; // corey said make constants B^(
  private final TalonFX motorTurret = new TalonFX(Constants.TurretCanID, "rio");
  private final TalonFX motorKicker = new TalonFX(Constants.KickerCanID, "rio");
  private final TalonFX motorShooterWheels = new TalonFX(Constants.ShooterWheelsCanID, "rio");
  public final TalonFX motorShooterArm = new TalonFX(Constants.ShooterElevationMotorCanID, "rio");

  private double armAngle = 0.075;

  public static enum AimTarget {
    AUTO,
    LEFT,
    RIGHT
  }
 
  private AimTarget Target = AimTarget.AUTO;
  public Optional<Alliance> ally;

  /** Creates a new TurretSubsystem. */
  public ShooterSubsystem(CommandSwerveDrivetrain T_driveTrain) {
    this.T_driveTrain = T_driveTrain;

    ally = DriverStation.getAlliance();

    }

  public void SetTarget(AimTarget NewTarget) {
    Target = NewTarget;
  }

  public void StartREV() { // JEFF DOESNT LIKE THE NAME
    double KickerSpeed = Constants.kickerOnSpeed;
    double ShooterSpeed = Constants.shooterOnSpeed;

    final VelocityVoltage m_kickerRequest = new VelocityVoltage(KickerSpeed).withSlot(0); // COREY SAID COULD BE MEMBER VARIBLES
    final VelocityVoltage m_shooterRequest = new VelocityVoltage(ShooterSpeed).withSlot(0);
    motorKicker.setControl(m_kickerRequest.withVelocity(KickerSpeed));
    motorShooterWheels.setControl(m_shooterRequest.withVelocity(ShooterSpeed));
  }

  public void StopREV() {
    final VelocityVoltage m_kickerRequest = new VelocityVoltage(Constants.kickerOffSpeed).withSlot(0);
    final VelocityVoltage m_shooterRequest = new VelocityVoltage(Constants.shooterOffSpeed).withSlot(0);
    motorKicker.setControl(m_kickerRequest.withVelocity(Constants.kickerOffSpeed));
    motorShooterWheels.setControl(m_shooterRequest.withVelocity(Constants.shooterOffSpeed));
  }

  public double CalculateShooterElevation(double Distance) {
    return 346132 + -19997 * Distance + 504 * Math.pow(Distance, 2) + -7.26 *Math.pow(Distance, 3) + 0.0652 * Math.pow(Distance, 4) + -3.74E-04 * Math.pow(Distance, 5) + 1.34E-06 * Math.pow(Distance, 6) + -2.73E-09 * Math.pow(Distance, 7) + 2.43E-12 * Math.pow(Distance, 8);   //calculated on 2/17
  }

  @Override
  public void periodic() {

    StatusSignal <Angle> motorPose = motorTurret.getPosition();
    SmartDashboard.putNumber("Turret position", motorPose.getValueAsDouble());
    //System.out.println(motorPose.getValueAsDouble()); // JeFf DoEsNt LiKe ThIs CoMmEnT // jEfF dOeSnT lIkE tHiS cOmMeNt // if you couldnt tell brodie was here

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

    double zDistance = Math.pow(yDifference, 2) + Math.pow(xDifference, 2);

    // Calculates the turret angle for the target in rads
    double turretAngleGlobal = -(Math.atan2(yDifference, xDifference)) + RobotYawRad;
    SmartDashboard.putNumber("rad Turret Angle Red Hub", turretAngleGlobal);

    // Converts the turret angle in rads to motor rotation
    double rotations = turretAngleGlobal / (2 * Math.PI);

    // This is setting the position in rotations, so pass the converted value in.
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0); //leave pos blank
    motorTurret.setControl(m_request.withPosition(rotations));
    
    SmartDashboard.putNumber("Turret angle setpoint", rotations);
    SmartDashboard.putNumber("PID output", motorTurret.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Shooter elevation angle", CalculateShooterElevation(zDistance));

    //CalculateShooterElevation(1);
    final PositionVoltage m_elevationRequest = new PositionVoltage(CalculateShooterElevation(zDistance)).withSlot(0);
    motorShooterArm.setControl(m_elevationRequest.withPosition(CalculateShooterElevation(zDistance)));
  }
}