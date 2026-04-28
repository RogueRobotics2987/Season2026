// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ApriltagSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.UtilitiesSubsystem;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.LowerShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeInCommand;
import frc.robot.commands.RaiseShooterCommand;


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
    private final CommandXboxController auxJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final ApriltagSubsystem visionSubsystem = new ApriltagSubsystem(drivetrain);

    private final ShooterSubsystem turretSubsystem = new ShooterSubsystem(drivetrain);

    public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

    public final IndexSubsystem m_IndexSubsystem = new IndexSubsystem();

    private final UtilitiesSubsystem m_UtilitiesSubsystem = new UtilitiesSubsystem(joystick, auxJoystick);

    private final SendableChooser<Command> autoChooser;

    public SlewRateLimiter filter = new SlewRateLimiter(8); // 8 / s

    private boolean brakeEnabled = false;
    public RobotContainer() {

        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        // SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    void disableApriltagAngle(){
        visionSubsystem.disableApriltagAngle();
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

        joystick.a().onTrue(Commands.runOnce(() -> {brakeEnabled = !brakeEnabled;}));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //TODO: TEST INTAKE IN AND OUT, POSITIONS, AND VOLTAGE TOLERANCES. <- Done?
        //TODO: (Once done with the intake feed function) Test intake feed function.
       
        joystick.leftTrigger().whileTrue(m_IntakeSubsystem.getIntakeOutCommand());
        joystick.leftTrigger().onFalse(m_IntakeSubsystem.getIntakeInCommand());
        
        //joystick.leftBumper().onTrue(m_IntakeSubsystem.runOnce(m_IntakeSubsystem::intakeReverse));


        auxJoystick.rightBumper()
            .whileTrue(turretSubsystem.getRunBeltsBackwardsCommand()
            .alongWith(m_IndexSubsystem.getStartBackwardsCommand()));
        auxJoystick.rightBumper()
            .onFalse(turretSubsystem.runOnce(turretSubsystem::stopKicker)
            .alongWith(m_IndexSubsystem.runOnce(m_IndexSubsystem::stop)));

        auxJoystick.rightTrigger().onTrue(m_IndexSubsystem.runOnce(m_IndexSubsystem::start));
        auxJoystick.rightTrigger().onFalse(m_IndexSubsystem.runOnce(m_IndexSubsystem::stop));
       
        auxJoystick.leftTrigger().onTrue(turretSubsystem.runOnce(turretSubsystem::StartREV));
        auxJoystick.leftTrigger().onFalse(turretSubsystem.runOnce(turretSubsystem::StopREV)); 

        auxJoystick.povUp().whileTrue(turretSubsystem.run(turretSubsystem::ShooterTrimUp));
        auxJoystick.povDown().whileTrue(turretSubsystem.run(turretSubsystem::ShooterTrimDown));
        auxJoystick.start().whileTrue(turretSubsystem.run(turretSubsystem::ResetShooterTrim));
        
        auxJoystick.povLeft().whileTrue(turretSubsystem.run(turretSubsystem::TurretTrimLeft));
        auxJoystick.povRight().whileTrue(turretSubsystem.run(turretSubsystem::TurretTrimRight));
        auxJoystick.back().whileTrue(turretSubsystem.run(turretSubsystem::ResetTurretTrim));

        auxJoystick.a().onTrue(turretSubsystem.runOnce(() -> turretSubsystem.DisableShooter()));
        auxJoystick.a().onFalse(turretSubsystem.runOnce(() -> turretSubsystem.EnableShooter()));

        auxJoystick.x().onTrue(turretSubsystem.runOnce(() -> turretSubsystem.SetTarget(ShooterSubsystem.AimTarget.LEFT)));
        auxJoystick.b().onTrue(turretSubsystem.runOnce(() -> turretSubsystem.SetTarget(ShooterSubsystem.AimTarget.RIGHT)));
        auxJoystick.y().onTrue(turretSubsystem.runOnce(() -> turretSubsystem.SetTarget(ShooterSubsystem.AimTarget.AUTO)));
        
        
        drivetrain.registerTelemetry(logger::telemeterize);

        addRumbleTrigger(Constants.shift1_05, 1);
        addRumbleTrigger(Constants.shift2_10, 0);
        addRumbleTrigger(Constants.shift2_05, 1);
        addRumbleTrigger(Constants.shift3_10, 0);
        addRumbleTrigger(Constants.shift3_05, 1);
        addRumbleTrigger(Constants.shift4_10, 0);
        addRumbleTrigger(Constants.shift4_05, 1);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();  
    }

    private void addRumbleTrigger(double timeThreshold, int mode){
        new Trigger(() -> Timer.getMatchTime() <- timeThreshold)
            .onTrue(m_UtilitiesSubsystem.rumbleController(mode));
    }
}
