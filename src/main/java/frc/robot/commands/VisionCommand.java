package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class VisionCommand extends Command {
    private final CommandSwerveDrivetrain m_CommandSwerveSubsystem;
    private int id;
    private double txnc;
    private double tync;
    private double ta;
    private double distToCamera;
    private double distToRobot;
    private double ambiguity;
    
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public VisionCommand(CommandSwerveDrivetrain subsystem) {
      m_CommandSwerveSubsystem = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      //addRequirements(subsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

      id = 0;
      txnc = 0;             
      tync = 0;             
      ta = 0;                 
      distToCamera = 0;  
      distToRobot = 0;    
      ambiguity = 0;


    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {    

      RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        for (RawFiducial fiducial : fiducials) {
          id = fiducial.id;                    // Tag ID
          txnc = fiducial.txnc;             // X offset (no crosshair)
          tync = fiducial.tync;             // Y offset (no crosshair)
          ta = fiducial.ta;                 // Target area
          distToCamera = fiducial.distToCamera;  // Distance to camera
          distToRobot = fiducial.distToRobot;    // Distance to robot
          ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
        }

      double Tx = LimelightHelpers.getTX ("limelight");
      double Ta = LimelightHelpers.getTA ("limelight");
      double Ty = LimelightHelpers.getTY ("limelight");

      SmartDashboard.putNumber("Ty: ", Ty);
      SmartDashboard.putNumber("Tx: ", Tx);   
      SmartDashboard.putNumber("Ta: ", Ta);

      SmartDashboard.putNumber("id:", id);
      SmartDashboard.putNumber("txnc:", txnc);
      SmartDashboard.putNumber("tync:", tync);
      SmartDashboard.putNumber("ta:", ta);
      SmartDashboard.putNumber("distToCamera:", distToCamera);
      SmartDashboard.putNumber("distToRobot:", distToRobot);
      SmartDashboard.putNumber("ambiguity:", ambiguity);

    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }


