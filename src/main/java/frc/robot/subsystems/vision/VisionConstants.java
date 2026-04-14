package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
  public static final String CAMERA_1_NAME = "Arducam3-front-10913"; // DEBUG:CAMERA_1_NAME
  public static final String CAMERA_2_NAME = "Arducam1-frontleft-10913"; // DEBUG:CAMERA_2_NAME
  public static final String CAMERA_3_NAME = "Arducam2-back-10913"; // DEBUG:CAMERA_3_NAME

  static final double camera_h = Units.inchesToMeters(16.875); // DEBUG:CAMERA_HEIGHT
  static final double camera_x = Units.inchesToMeters(9.5); // DEBUG:CAMERA_X_OFFSET
  static final double camera_y = Units.inchesToMeters(15.75); // DEBUG:CAMERA_Y_OFFSET

  // Robot to camera transforms (X = forward, Y = left in WPILib)
  public static final Transform3d ROBOT_TO_CAMERA_1 =
      new Transform3d(
          camera_y,
          camera_x,
          camera_h,
          new Rotation3d(0, (Math.PI) / 12, -(Math.PI) / 18)); // DEBUG:CAMERA_1_TRANSFORM
  public static final Transform3d ROBOT_TO_CAMERA_2 =
      new Transform3d(
          -camera_y,
          camera_x,
          camera_h,
          new Rotation3d(0, (Math.PI) / 12, (Math.PI) / 18)); // DEBUG:CAMERA_2_TRANSFORM
  public static final Transform3d ROBOT_TO_CAMERA_3 =
      new Transform3d(
          Units.inchesToMeters(10.875),
          -Units.inchesToMeters(9.0),
          Units.inchesToMeters(17.0),
          new Rotation3d((Math.PI) / 2, 0, 0)); // DEBUG:CAMERA_3_TRANSFORM

  public static final double MAX_PITCH_ROLL_DEGREES = 3.0; // DEBUG:VISION_MAX_TILT

  // Field dimensions and safeguards (meters)
  public static final double FIELD_LENGTH_METERS = 16.54; // DEBUG:FIELD_LENGTH
  public static final double FIELD_WIDTH_METERS = 9.14; // DEBUG:FIELD_WIDTH

  public static final double MAX_POSE_DIFFERENCE_METERS = 2.0; // DEBUG:VISION_MAX_POSE_DIFF

  public static final double MAX_AMBIGUITY = 0.2; // DEBUG:VISION_MAX_AMBIGUITY
}
