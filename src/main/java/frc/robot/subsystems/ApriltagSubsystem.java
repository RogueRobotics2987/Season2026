
package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class ApriltagSubsystem extends SubsystemBase {
  private CommandSwerveDrivetrain AT_driveTrain;
  private boolean apriltagAngle = true;
  private boolean rejectUpdate = false;
  private boolean rejectUpdateBack = false;
  private boolean rejectUpdateSide = false;
  
  private LimelightHelpers.PoseEstimate mt2_Front;
  private LimelightHelpers.PoseEstimate mt2_Back;
  private LimelightHelpers.PoseEstimate mt2_Side;
  private LimelightHelpers.PoseEstimate mt1;
  private LimelightHelpers.PoseEstimate mt1_Back;
  private LimelightHelpers.PoseEstimate mt1_Side;

  private final Field2d field = new Field2d();

  // Constructor for the subsystem, used for initial setup and instantiation of components.
  public ApriltagSubsystem(CommandSwerveDrivetrain AT_driveTrain) {
    this.AT_driveTrain = AT_driveTrain;
    this.apriltagAngle = true;
    SmartDashboard.putData("Field", field); 
  }

  public void disableApriltagAngle() {
    apriltagAngle = false;
  }

  /**
   * Called periodically whenever the CommandScheduler runs.
   * This is useful for "background" actions or logging data to the dashboard.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    try {
      mt2_Front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front"); // IP: 10.29.87.16
      mt2_Back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back"); // IP: 10.29.87.202
      mt2_Side = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-side"); // IP: 10.29.87.17
      // more code
      LimelightHelpers.SetRobotOrientation("limelight-front", AT_driveTrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight-back", AT_driveTrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight-side", AT_driveTrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

      rejectUpdate = mt2_Front == null || mt2_Front.tagCount == 0 ? true : false;

      if (!rejectUpdate) {
        if (apriltagAngle == true) {
          mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");

          AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,3));
          AT_driveTrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds); 
        } else {
            AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,99999999)); 
            AT_driveTrain.addVisionMeasurement(mt2_Front.pose, mt2_Front.timestampSeconds);

            // SmartDashboard.putNumber("Front Limelight X", mt2_Front.pose.getX());
            // SmartDashboard.putNumber("Front LimelightY", mt2_Front.pose.getY());
            // SmartDashboard.putNumber("Front Limelight Rotation", mt2_Front.pose.getRotation().getDegrees());
        }
      }  

      rejectUpdateBack = mt2_Back == null || mt2_Back.tagCount == 0 ? true : false;

      if (!rejectUpdateBack) {
        if (apriltagAngle == true) {
          mt1_Back = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

          AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,3));
          AT_driveTrain.addVisionMeasurement(mt1_Back.pose, mt1_Back.timestampSeconds);
        } else {
          AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,99999999)); 
          AT_driveTrain.addVisionMeasurement(mt2_Back.pose, mt2_Back.timestampSeconds);
          // SmartDashboard.putNumber("Back Limelight X", mt2_Back.pose.getX());
          // SmartDashboard.putNumber("Back Limelight Y", mt2_Back.pose.getY());
        }
      }

      rejectUpdateSide = mt2_Side == null || mt2_Side.tagCount == 0 ? true : false;

      if (!rejectUpdateSide) {
        if (apriltagAngle == true) {
          mt1_Side = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-side");

          AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,3));
          AT_driveTrain.addVisionMeasurement(mt1_Side.pose, mt1_Side.timestampSeconds);
        } else {
          AT_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,99999999)); 
          AT_driveTrain.addVisionMeasurement(mt2_Back.pose, mt2_Side.timestampSeconds);
          // SmartDashboard.putNumber("Side Limelight X", mt2_Side.pose.getX());
          // SmartDashboard.putNumber("Side Limelight Y", mt2_Side.pose.getY());
        }
      }
    } catch(Exception e) {
      // System.out.println("Catch error: " + e.toString());
    }

    field.setRobotPose((AT_driveTrain.getState().Pose));
    SmartDashboard.putData("Pose", field);
  }
}
