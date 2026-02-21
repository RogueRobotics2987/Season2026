
package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class ApriltagSubsystem extends SubsystemBase {
 
// Components (e.g., motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  // Example private motor controller:
  // private final PWMVictorSPX m_motor = new PWMVictorSPX(4);

  private CommandSwerveDrivetrain AT_driveTrain;
  private boolean apriltagAngle = true;
  private boolean rejectUpdate = false;

  private final Field2d field = new Field2d();

  /** Creates a new ExampleSubsystem. */
  public ApriltagSubsystem(CommandSwerveDrivetrain AT_driveTrain) {
    this.AT_driveTrain = AT_driveTrain;
    this.apriltagAngle = true;
    SmartDashboard.putData("Field", field); 
    // Constructor for the subsystem, used for initial setup and instantiation of components.
  }

  public void disableApriltagAngle(){
    apriltagAngle = false;
  }
  /**
   * Called periodically whenever the CommandScheduler runs.
   * This is useful for "background" actions or logging data to the dashboard.
   */
  @Override
  public void periodic() {

    // System.out.println("Yeah Periodic");

    // This method will be called once per scheduler run.
    try {
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      // more code
      LimelightHelpers.SetRobotOrientation("limelight", AT_driveTrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

      // if (Math.abs(AT_driveTrain.get()) > 360) {
      //   rejectUpdate = true;
      // }

      if (mt2.tagCount == 0) {
        rejectUpdate = true;
      }

      else {
        rejectUpdate = false;
      }

      if (!rejectUpdate) {
        if(apriltagAngle == true){
          AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,3)); 
        } 
        else {
            AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,99999999)); 
            AT_driveTrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            // System.out.println("Yippee");

            SmartDashboard.putNumber("Limelight X", mt2.pose.getX());
            SmartDashboard.putNumber("LimelightY", mt2.pose.getY());
            SmartDashboard.putNumber("Limelight Rotation", mt2.pose.getRotation().getDegrees());
          
          }
      }  
    } catch(NullPointerException e){
      //System.out.println("Catch in mt2" + e.toString());
    }
    field.setRobotPose((AT_driveTrain.getState().Pose));
    SmartDashboard.putData("Pose", field);
    
  }
  // Public methods to control the subsystem's components (e.g., setting motor speeds, reading sensor data)
  // Example public method:
  // public void runMotor(double speed) {
  //   m_motor.set(speed);
  // }
}
