
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
  LimelightHelpers.PoseEstimate front =
      LimelightHelpers.getBotPoseEstimate_wpiBlue(
          "limelight-front"
      );

  if (front != null && front.tagCount > 0) {
    AT_driveTrain.resetPose(
    new edu.wpi.first.math.geometry.Pose2d(
        front.pose.getTranslation(),
        AT_driveTrain.getState().Pose.getRotation()
    )
);
  }

  field.setRobotPose(AT_driveTrain.getState().Pose);
}
}
