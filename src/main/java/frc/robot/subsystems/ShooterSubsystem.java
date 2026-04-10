// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
  private final TalonFX followerMotorShooterWheels = new TalonFX(Constants.FollowerWheelsCanID, "rio");

  private final SparkMax m_shooterArmMotor = new SparkMax(Constants.ShooterElevationMotorCanID, MotorType.kBrushless);
  private final SparkClosedLoopController m_shooterArmClosedLoopController = m_shooterArmMotor.getClosedLoopController();

  private VelocityVoltage m_kickerRequest;
  private VelocityVoltage m_shooterRequest;
  private StatusSignal <Angle> motorPose;
  private double RobotX;
  private double RobotY;
  private double RobotYawRad;
  private double TurretXGlobal;
  private double TurretYGlobal;
  private double targetX = 1;
  private double targetY = 0;
  private double xDifference;
  private double yDifference;
  private double zDistance;
  private double turretAngleGlobal;
  private double rotations;
  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  private SwerveDriveState swerveDriveState;
  private boolean Shoot = false;

  public double turretTrim = 0;
  public double shooterTrim = 0;
  private double armAngle = 0.032;

  public static enum AimTarget {
    AUTO,
    LEFT,
    RIGHT
  }

  private AimTarget Target = AimTarget.AUTO;

  public Optional<Alliance> ally;
  public boolean ShooterEnable = true;

  /** Creates a new TurretSubsystem. */
  public ShooterSubsystem(CommandSwerveDrivetrain T_driveTrain) {
    this.T_driveTrain = T_driveTrain;

    // Sets the follower motor. followerMotorShooterWheels follows motorShooterWheels.
    followerMotorShooterWheels.setControl(new Follower(motorShooterWheels.getDeviceID(), MotorAlignmentValue.Opposed));

    // SmartDashboard.putBoolean("Get Auto Aim Enabled", true);
    // SmartDashboard.putNumber("Kicker Speed", Constants.kickerOnSpeed);
    SmartDashboard.putNumber("Shooter Speed", Constants.shooterOnSpeed);
    SmartDashboard.putNumber("Shooter Arm Angle", armAngle);

    ally = DriverStation.getAlliance();
    //  SmartDashboard.putNumber("Shooter Arm Angle Setpoint", 0);
  }

  public void ShooterTrimUp(){
    shooterTrim += 0.025;
    // SmartDashboard.putNumber("Shooter Trim", shooterTrim);
  }

  public void ShooterTrimDown(){
    shooterTrim -= 0.025;
    // SmartDashboard.putNumber("Shooter Trim", shooterTrim);
  }

  public void ResetShooterTrim(){
    shooterTrim = 0;
    // SmartDashboard.putNumber("Shooter Trim", shooterTrim);
  }

  public void TurretTrimLeft(){
    turretTrim += 0.0001;
    // SmartDashboard.putNumber("Turret Trim", turretTrim);
  }

  public void TurretTrimRight(){
    turretTrim -= 0.0001;
    // SmartDashboard.putNumber("Turret Trim", turretTrim);
  }

  public void ResetTurretTrim(){
    turretTrim = 0;
    // SmartDashboard.putNumber("Turret Trim", turretTrim);
  }

  public void SetTarget(AimTarget NewTarget) {
    Target = NewTarget;
  }

  public void DisableShooter() {
    ShooterEnable = false;
  }

  public void EnableShooter() {
    ShooterEnable = true;
  }

  public void RunBeltsBackwards() {
    m_kickerRequest = new VelocityVoltage(Constants.kickerReverseSpeed).withSlot(0);
    motorKicker.setControl(m_kickerRequest.withVelocity(Constants.kickerReverseSpeed));
  }

  public void stopKicker() {
   motorKicker.setControl(m_kickerRequest.withVelocity(Constants.kickerOffSpeed));
  }

  public Command getRunBeltsBackwardsCommand() {
    return this.runOnce(() -> { RunBeltsBackwards(); });
  }

  // TODO: Try passing and see if we want/need a different shooter curve for passing.
  public static double CalculateShooterArmAngle(double Distance) {
    double[] coeffs = new double[] { -6.035101436075545e-8, 0.0000374052934185901, -0.00844782026836158, 0.846726337524652, -24.365613640898 }; // { -6.035101436077553e-8, 0.0000349912528441702, -0.00736187207442231, 0.688750116125717, -16.7063305092175 }; // { -4.099336833060573e-8, 0.0000239012179501468, -0.00503738332108565, 0.478908001536328, -9.84271516887961 }; // { -0.00031712809409377, 0.0914666401428832, 1.50742600505378 }; //{ -0.000214507983170006, 0.0766951684161614, 1.51713305455971 };
    double y = 0.0;//                new semi fake curve ^                                                                                                           untested fake curve ^                                                                                 orginal -10 in curve ^                                           orginal curve ^
    for (int i = 0; i < coeffs.length; i++) {
      y = y * Distance + coeffs[i];
    }
    return y;
  }

  public static double CalculateShooterWheelSpeed(double Distance) {
    Distance = Math.abs(Distance);
    double[] coeffs = new double[] { 0.00000269098598337347, -0.00128183799256672, 0.397407382298784, 12.0197750357969 }; // { -4.019650136329579e-9, 0.000002714566323836, -0.000715285651934125, 0.0915898387077836, -5.47388226056847, 157.133466347824 }; // { -1.687528626629400e-7, 0.0000866148330315965, -0.0167943991239346, 1.64193708461437, -21.8448031293099 }; // { 0.0000107351997489714, -0.00406788058148555, 0.704035863776255, 3.65076121531093 }; // { 0.000614828163898388, 0.05264576064905, 29.534836246623 };
    double y = 0.0; //                                                                                                                                                                                                                                                           new semi fake curve ^                                                                                                           untested fake curve ^                                                         orginal curve or -10 in curve ^
    for (int i = 0; i < coeffs.length; i++) {
      y = y * Distance + coeffs[i];
    }
    return y;
  }

  public void StartREV() { // JEFF DOESNT LIKE THE NAME
    Shoot = true;
    double KickerSpeed = Constants.kickerOnSpeed; // SmartDashboard.getNumber("Kicker Speed", Constants.kickerOnSpeed); [This is used to manualy control the speed]

    m_kickerRequest = new VelocityVoltage(KickerSpeed).withSlot(0);
    motorKicker.setControl(m_kickerRequest.withVelocity(KickerSpeed));
  }

  public void StopREV() {
    Shoot = false;
    m_kickerRequest = new VelocityVoltage(Constants.kickerOffSpeed).withSlot(0);
    m_shooterRequest = new VelocityVoltage(Constants.shooterOffSpeed).withSlot(0);
    motorKicker.setControl(m_kickerRequest.withVelocity(Constants.kickerOffSpeed));
    motorShooterWheels.setControl(m_shooterRequest.withVelocity(Constants.shooterOffSpeed));
  }

  @Override
  public void periodic() {
    motorPose = motorTurret.getPosition();
    SmartDashboard.putNumber("Turret position", motorPose.getValueAsDouble());
    //System.out.println(motorPose.getValueAsDouble()); // JeFf DoEsNt LiKe ThIs CoMmEnT // jEfF dOeSnT lIkE tHiS cOmMeNt // if you couldn't tell brodie was here

    swerveDriveState = T_driveTrain.getState();

    // Gets Robot X, Y, Yaw
    RobotX = swerveDriveState.Pose.getX();
    RobotY = swerveDriveState.Pose.getY();
    RobotYawRad = swerveDriveState.Pose.getRotation().getRadians();

    // Calculates the global postion of the turret anywhere on the field
    TurretXGlobal = RobotX + Constants.turretOffsetH * Math.cos(RobotYawRad + Constants.turretOffsetAngleRad);
    TurretYGlobal = RobotY + Constants.turretOffsetH * Math.sin(RobotYawRad + Constants.turretOffsetAngleRad);
    // SmartDashboard.putNumber("YawRad", RobotYawRad);

    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red){
        targetX = Constants.redHubX;
        targetY = Constants.redHubY;
        // Constants.shooterOnSpeed = 40;

        if (Target == AimTarget.LEFT) {
          targetX = Constants.redPassLeftX;
          targetY = Constants.redPassLeftY;
          // Constants.shooterOnSpeed = 30;
        }

        if (Target == AimTarget.RIGHT) {
          targetX = Constants.redPassRightX;
          targetY = Constants.redPassRightY;
          // Constants.shooterOnSpeed = 30;
        }
      }

      if (ally.get() == Alliance.Blue) {
        targetX = Constants.blueHubX;
        targetY = Constants.blueHubY;
        // Constants.shooterOnSpeed = 40;
        
        if (Target == AimTarget.RIGHT) {
          targetX = Constants.bluePassRightX;
          targetY = Constants.bluePassRightY;
          // Constants.shooterOnSpeed = 30;
        }

        if (Target == AimTarget.LEFT) {
          targetX = Constants.bluePassLeftX;
          targetY = Constants.bluePassLeftY;
          // Constants.shooterOnSpeed = 30;
        }
      }
    } else {
          ally = DriverStation.getAlliance();
    }

    // Calculates the difference in the X, Y for the target
    xDifference = targetX - TurretXGlobal;
    yDifference = targetY - TurretYGlobal;

    zDistance = 39.3701 * Math.sqrt(Math.pow(yDifference, 2) + Math.pow(xDifference, 2));

    // Calculates the turret angle for the target in rads
    turretAngleGlobal = -(Math.atan2(yDifference, xDifference)) + RobotYawRad;
    // SmartDashboard.putNumber("rad Turret Angle Red Hub", turretAngleGlobal);

    // Converts the turret angle in rads to motor rotation
    rotations = turretAngleGlobal / (2 * Math.PI);
    
    rotations = rotations < 0 ? rotations + 1 : rotations;
    
    // TODO: Go through with drivers/Calise what they want to have up on elastic during comp.
    // SmartDashBoard Stuff 
    // SmartDashboard.putNumber("Rotaions", rotations);

    // SmartDashboard.putNumber("Shooter Trim", shooterTrim);
    // SmartDashboard.putNumber("Turret Trim", turretTrim);

    // SmartDashboard.putNumber("PID output", motorTurret.getClosedLoopOutput().getValueAsDouble());

    // SmartDashboard.putNumber("Shooter elevation angle", CalculateShooterElevation(zDistance));

    // SmartDashboard.putNumber("Z Distance to Hub", zDistance);
    // SmartDashboard.putNumber("Y Difference", yDifference);
    // SmartDashboard.putNumber("X Difference", xDifference);

    // SmartDashboard.putNumber("Target X", targetX);
    // SmartDashboard.putNumber("Target Y", targetY);

    // SmartDashboard.putNumber("Turret X Global", TurretXGlobal);
    // SmartDashboard.putNumber("Turret Y Global", TurretYGlobal);

    // SmartDashboard.putNumber("Robot X", RobotX);
    // SmartDashboard.putNumber("Robot Y", RobotY);

    // SmartDashboard.putString("Aim Target", Target.name());

    
    if(ShooterEnable == true) {
      double elevationAngleRequest = CalculateShooterArmAngle(zDistance) + shooterTrim;
      // double elevationAngleRequest = SmartDashboard.getNumber("Shooter Arm Angle", armAngle); // [This is used to manualy control the Shooter Arm Angle]
      if (Shoot == true) {
        double ShooterSpeed = CalculateShooterWheelSpeed(zDistance); // SmartDashboard.getNumber("Shooter Speed", Constants.shooterOnSpeed); [This is used to manualy control the speed]
        ShooterSpeed = Math.abs(ShooterSpeed);
        m_shooterRequest = new VelocityVoltage(ShooterSpeed).withSlot(0);
        motorShooterWheels.setControl(m_shooterRequest.withVelocity(ShooterSpeed));
      }

      m_shooterArmClosedLoopController.setSetpoint(elevationAngleRequest, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      motorTurret.setControl(m_request.withPosition(rotations + turretTrim)); // TODO: Fix turret aiming error of being either to far left or right.
    }

    if(ShooterEnable == false) {
      m_shooterArmClosedLoopController.setSetpoint(Constants.shooterArmDisable, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      motorTurret.setControl(m_request.withPosition(0));
    }
  }
}