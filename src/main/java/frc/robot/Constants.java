
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
    public static final double redPassLeftY = 1.500;

    // The X, Y coordinates for Red Passing Right in meters
    public static final double redPassRightX = 14.500;
    public static final double redPassRightY = 6.600;

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
    public static final double turretOffsetX = -155575; // set to 0.00635 for roomba testing
    public static final double turretOffsetY = 0.1016; // set to -0.10695 for roomba testing
    public static final double turretOffsetH = 0.20263; // these three values are calculated from turretOffsetX and TurretOffsetY
    public static final double turretOffsetAngle = 31.971;
    public static final double turretOffsetAngleRad = 2.5836;

    // Can IDs
    public static final int intakeAngleArmMotorCanID = 15;
    public static final int intakeAngleArmCanCoderCanID = 23;
    public static final int intakeArmWheelMotorCanID = 16;

    public static final int indexMotorCanID = 22;
    public static final int TurretCanID = 20;
    public static final int KickerCanID = 17;

    public static final int ShooterElevationMotorCanID = 9; //neo 550
    public static final int ShooterElevationCanCoderCanID = 21;
    public static final int ShooterWheelsCanID = 18;
    public static final int FollowerWheelsCanID = 19;

    // PID setpoints
    public static final double intakeOutAngle = 0.243;
    public static final double intakeInAngle  = 0;

    public static final double intakeStartSpeed = 0.725; //0.6;
    public static final double intakeReverseSpeed = -0.725; //-0.6;
    public static final double intakeStopSpeed = 0;

    public static final double indexOnSpeed = 75; //75
    public static final double indexOffSpeed = 0;

    public static final double kickerOnSpeed = 75; //75
    public static final double kickerOffSpeed = 0;

    public static double shooterOnSpeed = 40; //40
    public static final double shooterOffSpeed = 0;

    public static final double shooterArmDisable = 0;

    public static final double deadband = 0.03;

    public static final double gasPedalLimit = 0.25;

    //variables for the match times where we want the controller to rumble
    public static final int shift1_05 = 135;
    public static final int shift2_10 = 115;
    public static final int shift2_05 = 110;
    public static final int shift3_10 = 90;
    public static final int shift3_05 = 85;
    public static final int shift4_10 = 65;
    public static final int shift4_05 = 60;

    public static final double armCurrentThreshold = 15.0; //TODO: tune this
    public static final int overCurrentCycle = 5;
    public static final double intakeArmPositionTolerance = 0.05; //TODO: tune this
}