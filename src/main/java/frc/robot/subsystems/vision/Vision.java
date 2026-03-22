package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class Vision extends SubsystemBase {
  private final DriveSubsystem m_driveSubsystem;

  private AprilTagFieldLayout m_fieldLayout;

  private final PhotonCamera m_camera1;
  private final PhotonCamera m_camera2;
  private final PhotonCamera m_camera3;

  private PhotonPoseEstimator m_estimator1;
  private PhotonPoseEstimator m_estimator2;
  private PhotonPoseEstimator m_estimator3;

  public Vision(DriveSubsystem driveSubsystem, AprilTagFieldLayout fieldLayout) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_fieldLayout = fieldLayout;

    m_camera1 = new PhotonCamera(VisionConstants.CAMERA_1_NAME);
    m_camera2 = new PhotonCamera(VisionConstants.CAMERA_2_NAME);
    m_camera3 = new PhotonCamera(VisionConstants.CAMERA_3_NAME);

    if (m_fieldLayout != null) {
      m_estimator1 = new PhotonPoseEstimator(m_fieldLayout, VisionConstants.ROBOT_TO_CAMERA_1);
      m_estimator2 = new PhotonPoseEstimator(m_fieldLayout, VisionConstants.ROBOT_TO_CAMERA_2);
      m_estimator3 = new PhotonPoseEstimator(m_fieldLayout, VisionConstants.ROBOT_TO_CAMERA_3);
    }
  }

  @Override
  public void periodic() {
    // Process estimates
    processEstimator(m_estimator1, m_camera1);
    processEstimator(m_estimator2, m_camera2);
    processEstimator(m_estimator3, m_camera3);
  }

  private void processEstimator(PhotonPoseEstimator estimator, PhotonCamera camera) {
    String prefix = "Vision/" + camera.getName() + "/";

    if (estimator == null) {
      SmartDashboard.putString(prefix + "Status", "No estimator");
      return;
    }
    if (!camera.isConnected()) {
      SmartDashboard.putString(prefix + "Status", "Not connected");
      return;
    }

    var results = camera.getAllUnreadResults();
    SmartDashboard.putNumber(prefix + "ResultCount", results.size());
    if (results.isEmpty()) {
      SmartDashboard.putString(prefix + "Status", "No results");
      return;
    }

    var pitch = Math.abs(m_driveSubsystem.getPitch());
    var roll = Math.abs(m_driveSubsystem.getRoll());

    if (pitch > VisionConstants.MAX_PITCH_ROLL_DEGREES
        || roll > VisionConstants.MAX_PITCH_ROLL_DEGREES) {
      SmartDashboard.putBoolean("Vision/IgnoringDueToPitchRoll", true);
      SmartDashboard.putString(prefix + "Status", "Rejected: pitch/roll");
      return;
    }
    SmartDashboard.putBoolean("Vision/IgnoringDueToPitchRoll", false);

    for (var result : results) {
      SmartDashboard.putBoolean(prefix + "HasTargets", result.hasTargets());
      SmartDashboard.putNumber(prefix + "TargetCount", result.getTargets().size());

      // Try coprocessor multi-tag first (real robot), fall back to lowest ambiguity (sim/single
      // tag)
      var estimatedPose = estimator.estimateCoprocMultiTagPose(result);
      SmartDashboard.putBoolean(prefix + "CoprocMultiTag", estimatedPose.isPresent());

      if (estimatedPose.isEmpty()) {
        estimatedPose = estimator.estimateLowestAmbiguityPose(result);
        SmartDashboard.putBoolean(prefix + "LowestAmbiguity", estimatedPose.isPresent());
      }

      if (estimatedPose.isPresent()) {
        var est = estimatedPose.get();
        Pose2d pose2d = est.estimatedPose.toPose2d();
        SmartDashboard.putString(prefix + "EstPose", pose2d.toString());

        if (isVisionPoseValid(pose2d, m_driveSubsystem.getPose())) {
          m_driveSubsystem.addVisionMeasurement(pose2d, est.timestampSeconds);
          m_driveSubsystem.getField().getObject("VisionPose_" + camera.getName()).setPose(pose2d);
          SmartDashboard.putString(prefix + "Status", "Applied");
        } else {
          SmartDashboard.putString(prefix + "Status", "Rejected: validation");
        }
      } else {
        SmartDashboard.putString(prefix + "Status", "No pose estimate");
      }
    }
  }

  /**
   * Basic results validator: 1. Must be inside the field bounds (with a 0.5m buffer) 2. Prevent
   * "light speed" teleportation by rejecting poses too far from current odometry
   *
   * @param visionPose The newly estimated 2D pose from PhotonVision
   * @param currentPose The 2D pose from the DriveSubsystem odometry
   * @return true if the pose passes all sanity checks
   */
  private boolean isVisionPoseValid(Pose2d visionPose, Pose2d currentPose) {
    boolean insideField =
        visionPose.getX() >= -0.5
            && visionPose.getX() <= VisionConstants.FIELD_LENGTH_METERS + 0.5
            && visionPose.getY() >= -0.5
            && visionPose.getY() <= VisionConstants.FIELD_WIDTH_METERS + 0.5;

    // Skip the jump check when disabled — allows vision to establish the correct
    // starting position when the robot is first placed on the field.
    boolean jumpTooLarge =
        !DriverStation.isDisabled()
            && visionPose.getTranslation().getDistance(currentPose.getTranslation())
                > VisionConstants.MAX_POSE_DIFFERENCE_METERS;

    if (!insideField || jumpTooLarge) {
      SmartDashboard.putString(
          "Vision/RejectedPoseReason", !insideField ? "Outside Field" : "Jump Too Large");
      return false;
    }

    return true;
  }

  public PhotonCamera getCamera1() {
    return m_camera1;
  }

  public PhotonCamera getCamera2() {
    return m_camera2;
  }

  public PhotonCamera getCamera3() {
    return m_camera3;
  }
}
