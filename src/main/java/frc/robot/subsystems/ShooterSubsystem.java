// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  public double turretTrim = 0;
  public double shooterTrim = 0;
  private double armAngle = 0.032; // remove?

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

    // Set the follower motor. followerMotorShooterWheels follows motorShooterWheels.
    followerMotorShooterWheels.setControl(new Follower(motorShooterWheels.getDeviceID(), MotorAlignmentValue.Opposed));

    SmartDashboard.putBoolean("Get Auto Aim Enabled", true);
    SmartDashboard.putNumber("Kicker Speed", Constants.kickerOnSpeed);
    SmartDashboard.putNumber("Shooter Speed", Constants.shooterOnSpeed);
    SmartDashboard.putNumber("Shooter Arm Angle", armAngle);

    ally = DriverStation.getAlliance();
     SmartDashboard.putNumber("Shooter Arm Angle Setpoint", 0);
  }

  public void ShooterTrimUp(){
    shooterTrim = shooterTrim + 0.0001;
    SmartDashboard.putNumber("Shooter Trim", shooterTrim);
  }

  public void ShooterTrimDown(){
    shooterTrim = shooterTrim - 0.0001;
    SmartDashboard.putNumber("Shooter Trim", shooterTrim);
  }

  public void ResetShooterTrim(){
    shooterTrim = 0;
    SmartDashboard.putNumber("Shooter Trim", shooterTrim);
  }

  public void TurretTrimLeft(){
    turretTrim = turretTrim + 0.0001;
    SmartDashboard.putNumber("Turret Trim", turretTrim);
  }

  public void TurretTrimRight(){
    turretTrim = turretTrim - 0.0001;
    SmartDashboard.putNumber("Turret Trim", turretTrim);
  }

  public void ResetTurretTrim(){
    turretTrim = 0;
    SmartDashboard.putNumber("Turret Trim", turretTrim);
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

  public void StartREV() { // JEFF DOESNT LIKE THE NAME
    double KickerSpeed = SmartDashboard.getNumber("Kicker Speed", Constants.kickerOnSpeed); //15?
    double ShooterSpeed = SmartDashboard.getNumber("Shooter Speed", Constants.shooterOnSpeed);

    m_kickerRequest = new VelocityVoltage(KickerSpeed).withSlot(0); // COREY SAID COULD BE MEMBER VARIBLES
    m_shooterRequest = new VelocityVoltage(ShooterSpeed).withSlot(0);
    motorKicker.setControl(m_kickerRequest.withVelocity(KickerSpeed));
    motorShooterWheels.setControl(m_shooterRequest.withVelocity(ShooterSpeed));
  }

  public void StopREV() {
    m_kickerRequest = new VelocityVoltage(Constants.kickerOffSpeed).withSlot(0);
    m_shooterRequest = new VelocityVoltage(Constants.shooterOffSpeed).withSlot(0);
    motorKicker.setControl(m_kickerRequest.withVelocity(Constants.kickerOffSpeed));
    motorShooterWheels.setControl(m_shooterRequest.withVelocity(Constants.shooterOffSpeed));
  }

  public double CalculateShooterElevation(double Distance) {
    // return 346132 + -19997 * Distance + 504 * Math.pow(Distance, 2) + -7.26 *Math.pow(Distance, 3) + 0.0652 * Math.pow(Distance, 4) + -3.74E-04 * Math.pow(Distance, 5) + 1.34E-06 * Math.pow(Distance, 6) + -2.73E-09 * Math.pow(Distance, 7) + 2.43E-12 * Math.pow(Distance, 8);   //calculated on 2/17
    // return 346132.119250917 + -19996.8582755326 * Distance + 504.49244271105 * Math.pow(Distance, 2) + -7.25940609155529 * Math.pow(Distance, 3) + 0.0651653318101579 * Math.pow(Distance, 4) + -0.000373679751502965 * Math.pow(Distance, 5) + 0.00000133674724588682 * Math.pow(Distance, 6) + -0.00000000272739454061091 * Math.pow(Distance, 7) + 2.43003670597437E-12 * Math.pow(Distance, 8);
    // return 55.3044635576478000000 + -6.36094409009439000000 * Distance + 0.32149925333374000000000000 * Math.pow(Distance, 2) + -0.00940821025540215 * Math.pow(Distance, 3) + 0.00017664574156125100000000 * Math.pow(Distance, 4) + -0.00000222537607870770000000 * Math.pow(Distance, 5) + 0.00000001906849698739800000 * Math.pow(Distance, 6) + -0.000000000109846770121158000 * Math.pow(Distance, 7) + 0.000000000000407556427043987000 * Math.pow(Distance, 8) + -0.000000000000000880327958574075 * Math.pow(Distance, 9) + 0.000000000000000000841471744895 * Math.pow(Distance, 10);
    // return -17658.1810607820000000000 + 1699.70405971566000000000 * Distance + -73.37785024238110000000000000 * Math.pow(Distance, 2) + 1.87084632575126000000000000 * Math.pow(Distance, 3) + -0.03119461289815020000000000 * Math.pow(Distance, 4) + 0.00035541645787583600000000 * Math.pow(Distance, 5) + -0.00000280208223621810000000 * Math.pow(Distance, 6) + 0.000000015093516911952800000000 * Math.pow(Distance, 7) + -0.00000000005315791777 * Math.pow(Distance, 8) + 0.000000000000110529019078551000 * Math.pow(Distance, 9) + -0.000000000000000103026272855651 * Math.pow(Distance, 10);
    // return -166.4705896121040000000 + 13.04197682346640000000 * Distance + -0.45355909682121900000000000 * Math.pow(Distance, 2) + 0.00921255553273902000000000 * Math.pow(Distance, 3) + -0.00012094532664654600000000 * Math.pow(Distance, 4) + 0.00000107180402830985000000 * Math.pow(Distance, 5) + -0.00000000649120754999035000 * Math.pow(Distance, 6) + 0.000000000026527023622102900000 * Math.pow(Distance, 7) + -0.000000000000070012638661799200 * Math.pow(Distance, 8) + 0.000000000000000107793069638857 * Math.pow(Distance, 9) + -0.000000000000000000073545521976 * Math.pow(Distance, 10);
    return (-244.2500163983830000000 + 18.33245249610840000000 * Distance + -0.61129148599016500000000000 * Math.pow(Distance, 2) + 0.01191623128273760000000000 * Math.pow(Distance, 3) + -0.00015029375962969700000000 * Math.pow(Distance, 4) + 0.00000128096529744011000000 * Math.pow(Distance, 5) + -0.00000000746980202580367000 * Math.pow(Distance, 6) + 0.000000000029425639296377200000 * Math.pow(Distance, 7) + -0.000000000000074946047543541000 * Math.pow(Distance, 8) + 0.000000000000000111470318010063 * Math.pow(Distance, 9) + -0.000000000000000000073545502934 * Math.pow(Distance, 10)) * 1000;
    // if (Distance <= 75){
    //   return 0;
    // }

    // else{
    //   return -201.0426277257740000000 + 12.39140187109030000000 * Distance + -0.32752075587002500000000000 * Math.pow(Distance, 2) + 0.00481306895081772000000000 * Math.pow(Distance, 3) + -0.00004223018170047660000000 * Math.pow(Distance, 4) + 0.00000021415550566587700000 * Math.pow(Distance, 5) + -0.00000000046497360883538700 * Math.pow(Distance, 6) + -0.000000000001012042357180700000 * Math.pow(Distance, 7) + 0.000000000000009256378555327140 * Math.pow(Distance, 8) + -0.000000000000000022959256981162 * Math.pow(Distance, 9) + 0.000000000000000000020789680208 * Math.pow(Distance, 10);
    // }
  }

  @Override
  public void periodic() {

    motorPose = motorTurret.getPosition();
    SmartDashboard.putNumber("Turret position", motorPose.getValueAsDouble());
    //System.out.println(motorPose.getValueAsDouble()); // JeFf DoEsNt LiKe ThIs CoMmEnT // jEfF dOeSnT lIkE tHiS cOmMeNt // if you couldnt tell brodie was here

    swerveDriveState = T_driveTrain.getState();

    // Gets Robot X, Y, Yaw
    RobotX = swerveDriveState.Pose.getX();
    RobotY = swerveDriveState.Pose.getY();
    RobotYawRad = swerveDriveState.Pose.getRotation().getRadians();

    // Calculates the global postion of the turret anywhere on the field
    TurretXGlobal = RobotX + Constants.turretOffsetH * Math.cos(RobotYawRad + Constants.turretOffsetAngleRad);
    TurretYGlobal = RobotY + Constants.turretOffsetH * Math.sin(RobotYawRad + Constants.turretOffsetAngleRad);
    SmartDashboard.putNumber("YawRad", RobotYawRad);

    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red){
        targetX = Constants.redHubX;
        targetY = Constants.redHubY;
        Constants.shooterOnSpeed = 50;

        if (Target == AimTarget.LEFT) {
          targetX = Constants.redPassLeftX;
          targetY = Constants.redPassLeftY;
          Constants.shooterOnSpeed = 40;
        }

        if (Target == AimTarget.RIGHT) {
          targetX = Constants.redPassRightX;
          targetY = Constants.redPassRightY;
          Constants.shooterOnSpeed = 40;
        }
      }

      if (ally.get() == Alliance.Blue) {
        targetX = Constants.blueHubX;
        targetY = Constants.blueHubY;
        Constants.shooterOnSpeed = 50;
        
        if (Target == AimTarget.RIGHT) {
          targetX = Constants.bluePassRightX;
          targetY = Constants.bluePassRightY;
          Constants.shooterOnSpeed = 40;
        }

        if (Target == AimTarget.LEFT) {
          targetX = Constants.bluePassLeftX;
          targetY = Constants.bluePassLeftY;
          Constants.shooterOnSpeed = 40;
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
    SmartDashboard.putNumber("rad Turret Angle Red Hub", turretAngleGlobal);

    // Converts the turret angle in rads to motor rotation
    rotations = turretAngleGlobal / (2 * Math.PI);
    
    if(rotations > 0.5) {
      rotations = rotations - 1;
    }
    
    // SmartDashBoard Stuff
    SmartDashboard.putNumber("Rotaions", rotations);

    SmartDashboard.putNumber("Shooter Trim", shooterTrim);
    SmartDashboard.putNumber("Turret Trim", turretTrim);

    SmartDashboard.putNumber("PID output", motorTurret.getClosedLoopOutput().getValueAsDouble());

    SmartDashboard.putNumber("Shooter elevation angle", CalculateShooterElevation(zDistance));

    SmartDashboard.putNumber("Z Distance to Hub", zDistance);
    SmartDashboard.putNumber("Y Difference", yDifference);
    SmartDashboard.putNumber("X Difference", xDifference);

    SmartDashboard.putNumber("Target X", targetX);
    SmartDashboard.putNumber("Target Y", targetY);

    SmartDashboard.putNumber("Turret X Global", TurretXGlobal);
    SmartDashboard.putNumber("Turret Y Global", TurretYGlobal);

    SmartDashboard.putNumber("Robot X", RobotX);
    SmartDashboard.putNumber("Robot Y", RobotY);

    SmartDashboard.putString("Aim Target", Target.name());

    
    if(ShooterEnable == true) {
      // double elevationAngleRequest = CalculateShooterElevation(zDistance) + shooterTrim;
      double elevationAngleRequest = SmartDashboard.getNumber("Shooter Arm Angle", armAngle);
      m_shooterArmClosedLoopController.setSetpoint(elevationAngleRequest, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      motorTurret.setControl(m_request.withPosition(rotations + turretTrim));
    }

    if(ShooterEnable == false) {
      m_shooterArmClosedLoopController.setSetpoint(Constants.shooterArmDisable, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      motorTurret.setControl(m_request.withPosition(0));
    }
  }
}