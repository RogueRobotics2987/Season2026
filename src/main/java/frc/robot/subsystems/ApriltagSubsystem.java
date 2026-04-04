
package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.time.*;
import frc.robot.Constants;

public class ApriltagSubsystem extends SubsystemBase {
 
// Components (e.g., motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  // Example private motor controller:
  // private final PWMVictorSPX m_motor = new PWMVictorSPX(4);

  private CommandSwerveDrivetrain AT_driveTrain;
  private boolean apriltagAngle = true;
  private boolean rejectUpdateLuke = false;
  private boolean rejectUpdateLauren = false;
  private boolean rejectUpdateLiberty = false;
  private Instant aprilTagLastSeen;

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
      LimelightHelpers.PoseEstimate mt2_Luke = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-luke");
      LimelightHelpers.PoseEstimate mt2_Lauren = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-lauren");
      LimelightHelpers.PoseEstimate mt2_Liberty = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-liberty");
      // more code
      LimelightHelpers.SetRobotOrientation("limelight-luke", AT_driveTrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight-lauren", AT_driveTrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight-liberty", AT_driveTrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

      // if (Math.abs(AT_driveTrain.get()) > 360) {
      //   rejectUpdate = true;
      // }
  
      if (mt2_Luke.tagCount == 0){ // limelight 4 Luke ip: 10.29.87.17
        rejectUpdateLuke = true;
      }
      else {
        rejectUpdateLuke = false;
      }

      if (!rejectUpdateLuke){ //limelight 4 Luke
        if (apriltagAngle == true){
          LimelightHelpers.PoseEstimate mt1_Luke = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-luke");

          AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,3));
          AT_driveTrain.addVisionMeasurement(mt1_Luke.pose, mt1_Luke.timestampSeconds);
        }
        else {
          AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,99999999)); 
          AT_driveTrain.addVisionMeasurement(mt2_Luke.pose, mt2_Luke.timestampSeconds);
          SmartDashboard.putNumber("Limelight Luke X", mt2_Luke.pose.getX());
          SmartDashboard.putNumber("Limelight Luke Y", mt2_Luke.pose.getY());
        }
      }

      if (mt2_Lauren.tagCount == 0){ // limelight 4 Lauren ip: 10.29.87.16
        rejectUpdateLauren = true;
      }
      else {
        rejectUpdateLauren = false;
      }
      if (!rejectUpdateLauren){ // limelight 4 Lauren
        if (apriltagAngle == true){
          LimelightHelpers.PoseEstimate mt1_Lauren = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-lauren");

          AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,3));
          AT_driveTrain.addVisionMeasurement(mt1_Lauren.pose, mt1_Lauren.timestampSeconds);
        }
        else {
          AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,99999999)); 
          AT_driveTrain.addVisionMeasurement(mt2_Lauren.pose, mt2_Lauren.timestampSeconds);
          SmartDashboard.putNumber("Limelight Lauren X", mt2_Lauren.pose.getX());
          SmartDashboard.putNumber("Limelight Lauren Y", mt2_Lauren.pose.getY());
        }
      }

      if (mt2_Liberty.tagCount == 0){ // limelight 4 Liberty ip: 10.29.87.202
        rejectUpdateLiberty = true;
      }
      else {
        rejectUpdateLiberty = false;
      }
      if (!rejectUpdateLiberty){ // limelight 4 Liberty
        if (apriltagAngle == true){
          LimelightHelpers.PoseEstimate mt1_Liberty = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-liberty");

          AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,3));
          AT_driveTrain.addVisionMeasurement(mt1_Liberty.pose, mt1_Liberty.timestampSeconds);
        }
        else {
          AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,99999999)); 
          AT_driveTrain.addVisionMeasurement(mt2_Liberty.pose, mt2_Liberty.timestampSeconds);
          SmartDashboard.putNumber("Limelight Liberty X", mt2_Liberty.pose.getX());
          SmartDashboard.putNumber("Limelight Liberty Y", mt2_Liberty.pose.getY());
        }
      }

    } catch(NullPointerException e){
      //System.out.println("Catch in mt2" + e.toString());
    }
    field.setRobotPose((AT_driveTrain.getState().Pose));
    SmartDashboard.putData("Pose", field);

    // if (rejectUpdateLuke == false || rejectUpdate == false){
    //   aprilTagLastSeen = Instant.now();
    // }
    // long millis = Duration.between(Instant.now(), aprilTagLastSeen).toMillis();
    // SmartDashboard.putString("April Tag Last Seen", "" + millis);
    
  }
  // Public methods to control the subsystem's components (e.g., setting motor speeds, reading sensor data)
  // Example public method:
  // public void runMotor(double speed) {
  //   m_motor.set(speed);
  // }
}
