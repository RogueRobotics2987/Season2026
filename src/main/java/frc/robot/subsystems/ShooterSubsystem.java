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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

  private double armAngle = 0.032;

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
    SmartDashboard.putBoolean("Get Auto Aim Enabled", true);
    SmartDashboard.putNumber("Kicker Speed", Constants.kickerOnSpeed);
    SmartDashboard.putNumber("Shooter Speed", Constants.shooterOnSpeed);
    enableLimitSwitch();



    ally = DriverStation.getAlliance();
     SmartDashboard.putNumber("Shooter Arm Angle Setpoint", 0);

  }

  public void SetTarget(AimTarget NewTarget) {
    Target = NewTarget;
  }

  public void StartREV() { // JEFF DOESNT LIKE THE NAME
    double KickerSpeed = SmartDashboard.getNumber("Kicker Speed", Constants.kickerOnSpeed);
    double ShooterSpeed = SmartDashboard.getNumber("Shooter Speed", Constants.shooterOnSpeed);

    

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
    // return 346132 + -19997 * Distance + 504 * Math.pow(Distance, 2) + -7.26 *Math.pow(Distance, 3) + 0.0652 * Math.pow(Distance, 4) + -3.74E-04 * Math.pow(Distance, 5) + 1.34E-06 * Math.pow(Distance, 6) + -2.73E-09 * Math.pow(Distance, 7) + 2.43E-12 * Math.pow(Distance, 8);   //calculated on 2/17
    // return 346132.119250917 + -19996.8582755326 * Distance + 504.49244271105 * Math.pow(Distance, 2) + -7.25940609155529 * Math.pow(Distance, 3) + 0.0651653318101579 * Math.pow(Distance, 4) + -0.000373679751502965 * Math.pow(Distance, 5) + 0.00000133674724588682 * Math.pow(Distance, 6) + -0.00000000272739454061091 * Math.pow(Distance, 7) + 2.43003670597437E-12 * Math.pow(Distance, 8);
    // return 55.3044635576478000000 + -6.36094409009439000000 * Distance + 0.32149925333374000000000000 * Math.pow(Distance, 2) + -0.00940821025540215 * Math.pow(Distance, 3) + 0.00017664574156125100000000 * Math.pow(Distance, 4) + -0.00000222537607870770000000 * Math.pow(Distance, 5) + 0.00000001906849698739800000 * Math.pow(Distance, 6) + -0.000000000109846770121158000 * Math.pow(Distance, 7) + 0.000000000000407556427043987000 * Math.pow(Distance, 8) + -0.000000000000000880327958574075 * Math.pow(Distance, 9) + 0.000000000000000000841471744895 * Math.pow(Distance, 10);
    return -8.75718395925721000000000000 + 0.57214947900302200000000000 * Distance + -0.01613001753504010000000000 * Math.pow(Distance, 2) + 0.00025623050606685500000000 * Math.pow(Distance, 3) + -0.00000250742431430320000000 * Math.pow(Distance, 4) + 0.000000015471366655073400000000 * Math.pow(Distance, 5) +  -0.000000000058743061130108700000 * Math.pow(Distance, 6) + 0.000000000000125415245199520000 * Math.pow(Distance, 7) + -0.000000000000000115224937543216 * Math.pow(Distance, 8);
  }

   public void enableLimitSwitch(){
    TalonFXConfiguration limitSwitch = new TalonFXConfiguration();
    limitSwitch.HardwareLimitSwitch.withReverseLimitAutosetPositionEnable(true);
    motorTurret.getConfigurator().apply(limitSwitch);
  }

  public void disableLimitSwitch(){
    TalonFXConfiguration limitSwitch = new TalonFXConfiguration();
    limitSwitch.HardwareLimitSwitch.withReverseLimitAutosetPositionEnable(false);
    motorTurret.getConfigurator().apply(limitSwitch);
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

    double zDistance = 39.3701 * Math.sqrt(Math.pow(yDifference, 2) + Math.pow(xDifference, 2)); //distance in inches

    // Calculates the turret angle for the target in rads
    double turretAngleGlobal = -(Math.atan2(yDifference, xDifference)) + RobotYawRad;
    SmartDashboard.putNumber("rad Turret Angle Red Hub", turretAngleGlobal);

    // Converts the turret angle in rads to motor rotation
    double rotations = turretAngleGlobal / (2 * Math.PI);
    
    if(rotations < 0) {
      rotations = rotations + 1;
    }

    // This is setting the position in rotations, so pass the converted value in.

    
    //SmartDashboard.putNumber("Turret angle setpoint", rotations);
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


    // CalculateShooterElevation(1);
    // PositionVoltage m_elevationRequest = new PositionVoltage(CalculateShooterElevation(zDistance)).withSlot(0);
    // motorShooterArm.setControl(m_elevationRequest.withPosition(CalculateShooterElevation(zDistance)));
    final PositionVoltage m_elevationRequest = new PositionVoltage(SmartDashboard.getNumber("Shooter Arm Angle Setpoint", 0)).withSlot(0);
    motorShooterArm.setControl(m_elevationRequest.withPosition(SmartDashboard.getNumber("Shooter Arm Angle Setpoint", 0)));
    // if(SmartDashboard.getBoolean("Get Auto Aim Enabled", false)) {
    //   final PositionVoltage m_request = new PositionVoltage(0).withSlot(0); //leave pos blank
    //   motorTurret.setControl(m_request.withPosition(rotations));
    //   m_elevationRequest = new PositionVoltage(SmartDashboard.getNumber("Shooter Arm Angle Setpoint", 0)).withSlot(0);
    //   motorShooterArm.setControl(m_elevationRequest.withPosition(SmartDashboard.getNumber("Shooter Arm Angle Setpoint", 0)));
    // }
    
  }
}