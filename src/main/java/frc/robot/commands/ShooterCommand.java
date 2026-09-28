// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */

  private final ShooterSubsystem m_shooterSubsystem;
  private final IndexSubsystem m_indexSubsystem;


  public ShooterCommand(ShooterSubsystem shooterSubsystem,
                        IndexSubsystem indexSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    m_indexSubsystem = indexSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //check to see if turret is at the angle its said to move to
    //if(Turretpose error <= #) { //make sure the turret PID error is small enough to make shot // line up blue or red
    m_shooterSubsystem.StartREV();
    m_indexSubsystem.start(); 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.StopREV();
    m_indexSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
