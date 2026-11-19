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
    RIGHT,
    None
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

  public void ShooterTrimUp() {
    shooterTrim += 0.05;
}

public void ShooterTrimDown() {
    shooterTrim -= 0.05;
}

public void ResetShooterTrim() {
    shooterTrim = 0;
}

  public void TurretTrimLeft(){
    turretTrim += .01;
    SmartDashboard.putNumber("Turret Trim", turretTrim);
  }

  public void TurretTrimRight(){
    turretTrim -= .01;
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

  // // TODO: Try passing and see if we want/need a different shooter curve for passing.
  // public static double CalculateShooterArmAngle(double Distance) {
  //   double[] coeffs = new double[] { -6.035101436075545e-8, 0.0000374052934185901, -0.00844782026836158, 0.846726337524652, -24.365613640898 }; // { -6.035101436077553e-8, 0.0000349912528441702, -0.00736187207442231, 0.688750116125717, -16.7063305092175 }; // { -4.099336833060573e-8, 0.0000239012179501468, -0.00503738332108565, 0.478908001536328, -9.84271516887961 }; // { -0.00031712809409377, 0.0914666401428832, 1.50742600505378 }; //{ -0.000214507983170006, 0.0766951684161614, 1.51713305455971 };
  //   double y = 0.0;//                new semi fake curve ^                                                                                                           untested fake curve ^                                                                                 orginal -10 in curve ^                                           orginal curve ^
  //   for (int i = 0; i < coeffs.length; i++) {
  //     y = y * Distance + coeffs[i];
  //   }
  //   return y;
  // }

  // public static double CalculateShooterWheelSpeed(double Distance) {
  //   Distance = Math.abs(Distance);
  //   double[] coeffs = new double[] { 0.00000269098598337347, -0.00128183799256672, 0.397407382298784, 12.0197750357969 }; // { -4.019650136329579e-9, 0.000002714566323836, -0.000715285651934125, 0.0915898387077836, -5.47388226056847, 157.133466347824 }; // { -1.687528626629400e-7, 0.0000866148330315965, -0.0167943991239346, 1.64193708461437, -21.8448031293099 }; // { 0.0000107351997489714, -0.00406788058148555, 0.704035863776255, 3.65076121531093 }; // { 0.000614828163898388, 0.05264576064905, 29.534836246623 };
  //   double y = 0.0; //                                                                                                                                                                                                                                                           new semi fake curve ^                                                                                                           untested fake curve ^                                                         orginal curve or -10 in curve ^
  //   for (int i = 0; i < coeffs.length; i++) {
  //     y = y * Distance + coeffs[i];
  //   }
  //   return y;
  // }

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

    // MANUAL HOOD CONTROL
    if (ShooterEnable == true) {

        double elevationAngleRequest = shooterTrim;// + armAngle;

        m_shooterArmClosedLoopController.setSetpoint(
            elevationAngleRequest,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );

        // SHOOTER WHEELS
        if (Shoot == true) {

            double ShooterSpeed = 25;

            m_shooterRequest = new VelocityVoltage(ShooterSpeed).withSlot(0);

            motorShooterWheels.setControl(
                m_shooterRequest.withVelocity(ShooterSpeed)
            );
        }
    }

    // // DISABLE HOOD
    // if (ShooterEnable == false) {

    //     m_shooterArmClosedLoopController.setSetpoint(
    //         Constants.shooterArmDisable,
    //         ControlType.kPosition,
    //         ClosedLoopSlot.kSlot0
    //     );
    // }

    // MANUAL TURRET CONTROL
    motorTurret.setControl(
        m_request.withPosition(turretTrim)
    );

        if(ShooterEnable == false) {
      m_shooterArmClosedLoopController.setSetpoint(Constants.shooterArmDisable, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      motorTurret.setControl(m_request.withPosition(0));
    }
    // MANUAL TURRET CONTROL
    motorTurret.setControl(m_request.withPosition(turretTrim));
}



// if (ShooterEnable == true) {

//     if (Shoot == true) {
//         double ShooterSpeed = 25;

//         m_shooterRequest = new VelocityVoltage(ShooterSpeed).withSlot(0);

//         motorShooterWheels.setControl(
//             m_shooterRequest.withVelocity(ShooterSpeed)
//         );
//     }
// }

// if (ShooterEnable == false) {
//     m_shooterArmClosedLoopController.setSetpoint(
//         Constants.shooterArmDisable,
//         ControlType.kPosition,
//         ClosedLoopSlot.kSlot0
   // );
}


