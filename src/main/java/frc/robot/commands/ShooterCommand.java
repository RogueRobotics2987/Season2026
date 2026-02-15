// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */

  private final ShooterSubsystem m_shooterSubsystem;
  private final SpindexSubsystem m_spindexSubsystem;


  public ShooterCommand(ShooterSubsystem shooterSubsystem,
                        SpindexSubsystem spindexSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    m_spindexSubsystem = spindexSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //check to see if turret is at the angle its said to move to
    //if(Turretpose error <= #) { //make sure the turret PID error is small enough to make shot // line up blue or red
    m_shooterSubsystem.StartREV();
    /* } else {
      //dont run if turret is not lined up
      }
    */
    try{
      Thread.sleep(2000);
    }catch(InterruptedException e){
      //run spindex
      m_spindexSubsystem.start();
    }
    //run spindex
    m_spindexSubsystem.start(); //need both?

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.StopREV();
    //stop spindex
    m_spindexSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
