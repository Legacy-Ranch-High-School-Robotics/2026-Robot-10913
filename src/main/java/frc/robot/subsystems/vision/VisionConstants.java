package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * VISION SUBSYSTEM OPERATION:
 *
 * <p>The vision subsystem uses PhotonVision and AprilTags to estimate the robot's position on the
 * field.
 *
 * <p>CAMERAS: 3x Arducam OV9281 global shutter cameras - Camera 1 (front): Faces forward for
 * speaker/amp targeting - Camera 2 (front-left): Angled for wider field of view - Camera 3 (back):
 * Faces backward for defense and repositioning
 *
 * <p>POSE ESTIMATION PROCESS: 1. Each camera detects AprilTags (fiducial markers) on the field 2.
 * PhotonVision calculates 3D pose based on tag positions and camera transforms 3. Vision subsystem
 * validates pose estimates (field bounds, jump detection, ambiguity) 4. Valid poses are fused with
 * wheel odometry in the drive subsystem 5. Result: More accurate robot position than wheels alone
 * (no drift)
 *
 * <p>MULTI-TAG vs SINGLE-TAG: - Multi-tag (coprocessor): Uses multiple AprilTags simultaneously
 * (most accurate) - Single-tag (fallback): Uses one tag with ambiguity filtering (less reliable)
 *
 * <p>VALIDATION CHECKS: - Field bounds: Reject poses outside the field perimeter - Jump detection:
 * Reject poses that teleport too far from current odometry - Pitch/roll filtering: Ignore vision
 * when robot is tipped (bumps, collisions) - Ambiguity threshold: Reject single-tag estimates with
 * flipped pose risk
 *
 * <p>TYPICAL OPERATION: 1. Robot moves around field while cameras continuously scan for AprilTags
 * 2. Vision subsystem processes estimates and publishes diagnostics to Elastic 3. Valid poses
 * update drive odometry for accurate auto-aim and autonomous 4. Residual telemetry shows difference
 * between vision and wheel odometry
 */
public final class VisionConstants {
  public static final String CAMERA_1_NAME = "Arducam3-front-10913"; // DEBUG:CAMERA_1_NAME
  public static final String CAMERA_2_NAME = "Arducam1-frontleft-10913"; // DEBUG:CAMERA_2_NAME
  public static final String CAMERA_3_NAME = "Arducam2-back-10913"; // DEBUG:CAMERA_3_NAME

  static final double camera_h = 0.69215; // DEBUG:CAMERA_HEIGHT
  static final double camera_x = 0.08255; // DEBUG:CAMERA_X_OFFSET
  static final double camera_y = 0.2921; // DEBUG:CAMERA_Y_OFFSET

  // Robot to camera transforms (X = forward, Y = left in WPILib)
  public static final Transform3d ROBOT_TO_CAMERA_1 =
      new Transform3d(
          camera_y, -camera_x, camera_h, new Rotation3d(0, 0, 0)); // DEBUG:CAMERA_1_TRANSFORM
  public static final Transform3d ROBOT_TO_CAMERA_2 =
      new Transform3d(
          camera_y, camera_x, camera_h, new Rotation3d(0, 0, 0)); // DEBUG:CAMERA_2_TRANSFORM
  public static final Transform3d ROBOT_TO_CAMERA_3 =
      new Transform3d(
          camera_y - 0.0889,
          0,
          camera_h,
          new Rotation3d(0, 0, Math.PI)); // DEBUG:CAMERA_3_TRANSFORM

  // Tolerance: Maximum pitch/roll angle (degrees) before rejecting vision poses.
  // When robot tips beyond this threshold (bumps, ramps), vision estimates become unreliable.
  // Vision updates are ignored until robot levels out to prevent bad pose injection.
  public static final double MAX_PITCH_ROLL_DEGREES = 3.0; // DEBUG:VISION_MAX_TILT

  // Field dimensions and safeguards (meters)
  public static final double FIELD_LENGTH_METERS = 16.54; // DEBUG:FIELD_LENGTH
  public static final double FIELD_WIDTH_METERS = 9.14; // DEBUG:FIELD_WIDTH

  // Tolerance: Maximum allowed difference (meters) between vision pose and current odometry.
  // Prevents "teleportation" from single bad vision frames (reflections, wrong tags, etc.).
  // If vision pose is >2m away from wheel odometry, it's rejected as likely erroneous.
  // This check is disabled when robot is first placed to allow vision to set initial pose.
  public static final double MAX_POSE_DIFFERENCE_METERS = 2.0; // DEBUG:VISION_MAX_POSE_DIFF

  // Tolerance: Maximum pose ambiguity for single-tag estimates (0.0 = perfect, 1.0 = terrible).
  // Ambiguity measures how confident PhotonVision is about the pose (risk of 180° flip).
  // Values >0.2 indicate the tag could be seen from multiple angles, causing bad estimates.
  // Multi-tag estimates bypass this check since they're inherently more reliable.
  public static final double MAX_AMBIGUITY = 0.2; // DEBUG:VISION_MAX_AMBIGUITY
}
