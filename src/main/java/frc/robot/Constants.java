
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    // The X, Y coordinates for the Red Hub in meters
    public static final double redHubX = 11.920;
    public static final double redHubY = 4.035;

    // The X, Y coordinates for Red Passing Left in meters
    public static final double redPassLeftX = 14.500;
    public static final double redPassLeftY = 6.600;

    // The X, Y coordinates for Red Passing Right in meters
    public static final double redPassRightX = 14.500;
    public static final double redPassRightY = 1.500;

    // The X, Y coordinates for the Blue Hub in meters
    public static final double blueHubX = 4.620;
    public static final double blueHubY = 4.035;

    // The X, Y coordinates for Blue Passing Left in meters
    public static final double bluePassLeftX = 2.000; 
    public static final double bluePassLeftY = 6.600; 

    // The X, Y coordinates for Blue Passing Right in meters
    public static final double bluePassRightX = 2.000;
    public static final double bluePassRightY = 1.500;

    
    // The X, Y turret offset on the robot in meters
    public static final double turretOffsetX = -0.17145; // set to 0.00635 for roomba testing
    public static final double turretOffsetY = -0.108; // set to -0.10695 for roomba testing

    // Mechanism gear ratios
    // public static final double spindexGearRatio = 20.0;
    // public static final double climberGearRatio = 25.0;
    // public static final double intakeArmGearRatio = 50.0;
    // public static final double kickerGearRatio = 1.0;
    // public static final double turretGearRatio = 15.0;
    // public static final double shooterArnGearRatio = 25.0;

    // Can IDs
    public static final int intakeAngleArmMotorCanID = 14;
    public static final int intakeAngleArmCanCoderCanID = 19;
    public static final int intakeArmWheelMotorCanID = 15;

    public static final int spindexMotorCanID = 16;

    public static final int ClimberCanID = 40;

    public static final int KickerCanID = 18;

    public static final int TurretCanID = 20;

    public static final int ShooterElevationMotorCanID = 9;
    public static final int ShooterElevationCanCoderCanID = 21;
    public static final int ShooterWheelsCanID = 17;


    // // The PID Controller for the Climber
    // public static final double climberKP = 20.0;
    // public static final double climberKI = 0;
    // public static final double climberKD = 0;

    // // The PID Controller for the Turret
    // public static final double turretKP = 25;
    // public static final double turretKI = 0;
    // public static final double turretKD = 0;

    // // The PID Controller for the Kicker
    // public static final double kickerKP = 0.1;
    // public static final double kickerKI = 0;
    // public static final double kickerKD = 0;

    // // The PID Controller for the Shooter Wheels
    // public static final double shooterWheelsKP = 0.1;
    // public static final double shooterWheelsKI = 0;
    // public static final double shooterWheelsKD = 0;

    // // The PID Controller for the Shooter Wheels
    // public static final double shooterArmKP = 5;
    // public static final double shooterArmKI = 0;
    // public static final double shooterArmKD = 0;

    // // The PID Controller for the Spindex
    // public static final double spindexKP = 5;
    // public static final double spindexKI = 0;
    // public static final double spindexKD = 0;

    // // The PID Controller for the intake
    // public static final double intake_kP = 5;
    // public static final double intake_kI = 0;
    // public static final double intake_kD = 0;

    // PID setpoints
    public static final double intakeOutAngle = 0.333;
    public static final double intakeInAngle  = 0;

    public static final double intakeStartSpeed = 0.5;
    public static final double intakeStopSpeed = 0;

    public static final double spindexOnSpeed = 15;
    public static final double spindexOffSpeed = 0;

    public static final double kickerOnSpeed = 35;
    public static final double kickerOffSpeed = 0;

    public static final double shooterOnSpeed = 35;
    public static final double shooterOffSpeed = 0;
}
