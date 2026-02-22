// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ApriltagSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpindexSubsystem;

import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double MinSpeed = MaxSpeed * Constants.deadband;
    private double MinAngularRate = MaxAngularRate * Constants.deadband;
    

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MinSpeed).withRotationalDeadband(MinAngularRate) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController AuxJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final ApriltagSubsystem visionSubsystem = new ApriltagSubsystem(drivetrain);

    private final ShooterSubsystem turretSubsystem = new ShooterSubsystem(drivetrain);

    public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

    public final SpindexSubsystem m_SpindexSubsystem = new SpindexSubsystem();

    private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

    public SlewRateLimiter filter = new SlewRateLimiter(8); // 8 / s

    private boolean brakeEnabled = false;
    public RobotContainer() {
        configureBindings();
    }

    void disableApriltagAngle(){
        visionSubsystem.disableApriltagAngle();
    }

    void disableLimitSwitch(){
        turretSubsystem.disableLimitSwitch();
    }

    void resetPose(){
        drivetrain.resetPose(drivetrain.getState().Pose);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double outputMultiplier = Math.min(joystick.getRightTriggerAxis() + Constants.gasPedalLimit, 1);
                double velocityX = joystick.getLeftY() * MaxSpeed * outputMultiplier;
                double velocityY = joystick.getLeftX() * MaxSpeed * outputMultiplier;
                double angularRate = -joystick.getRightX() * MaxAngularRate * outputMultiplier;

                if (brakeEnabled &&
                    Math.abs(velocityX) < MinSpeed &&
                    Math.abs(velocityY) < MinSpeed &&
                    Math.abs(angularRate) < MinAngularRate
                ){
                    return brake;
                } else {
                    return drive.withVelocityX(velocityX) // Drive forward with negative Y (forward)
                        .withVelocityY(velocityY) // Drive left with negative X (left)
                        .withRotationalRate(angularRate); // Drive counterclockwise with negative X (left)
                }
            }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        joystick.back().onTrue(drivetrain.runOnce(
            () -> 
                {
                    if(DriverStation.getAlliance().get() == Alliance.Blue) {
                        drivetrain.setOperatorPerspectiveForward(new Rotation2d(0));
                    }
                    else {
                        drivetrain.setOperatorPerspectiveForward(new Rotation2d(180));
                    }
                }
        )); //testing field oriented drive
        //joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric(drivetrain.getState().Pose.getRotation())));

        joystick.leftTrigger().onTrue(m_IntakeSubsystem.runOnce(m_IntakeSubsystem::intakeOut));
        joystick.leftTrigger().onFalse(m_IntakeSubsystem.runOnce(m_IntakeSubsystem::intakeIn));
        joystick.leftBumper().onTrue(m_IntakeSubsystem.runOnce(m_IntakeSubsystem::hopperOut));
        joystick.rightBumper().onTrue(m_IntakeSubsystem.runOnce(m_IntakeSubsystem::intakeIn));

        AuxJoystick.rightTrigger().onTrue(m_SpindexSubsystem.runOnce(m_SpindexSubsystem::start));
        AuxJoystick.rightTrigger().onFalse(m_SpindexSubsystem.runOnce(m_SpindexSubsystem::stop));
       
        AuxJoystick.leftTrigger().onTrue(turretSubsystem.runOnce(turretSubsystem::StartREV));
        AuxJoystick.leftTrigger().onFalse(turretSubsystem.runOnce(turretSubsystem::StopREV)); 

        AuxJoystick.povDown().onTrue(turretSubsystem.runOnce(() -> turretSubsystem.DisableShooter()));
        AuxJoystick.povDown().onFalse(turretSubsystem.runOnce(() -> turretSubsystem.EnableShooter()));

        AuxJoystick.leftBumper().onTrue(turretSubsystem.runOnce(() -> turretSubsystem.SetTarget(ShooterSubsystem.AimTarget.LEFT)));
        AuxJoystick.rightBumper().onTrue(turretSubsystem.runOnce(() -> turretSubsystem.SetTarget(ShooterSubsystem.AimTarget.RIGHT)));
        AuxJoystick.y().onTrue(turretSubsystem.runOnce(() -> turretSubsystem.SetTarget(ShooterSubsystem.AimTarget.AUTO)));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(drivetrain.getState().Pose.getRotation())),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(0.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
