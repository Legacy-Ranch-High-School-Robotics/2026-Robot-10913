package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public final class VisionConstants {
  public static final String CAMERA_1_NAME = "Arducam3-front-10913";
  public static final String CAMERA_2_NAME = "Arducam1-frontleft-10913";
  public static final String CAMERA_3_NAME = "Arducam2-back-10913";

  static final double camera_h = 0.69215;
  static final double camera_x = 0.08255;
  static final double camera_y = 0.2921;

  // Robot to camera transforms (X = forward, Y = left in WPILib)
  public static final Transform3d ROBOT_TO_CAMERA_1 =
      new Transform3d(camera_y, -camera_x, camera_h, new Rotation3d(0, 0, 0));
  public static final Transform3d ROBOT_TO_CAMERA_2 =
      new Transform3d(camera_y, camera_x, camera_h, new Rotation3d(0, 0, 0));
  public static final Transform3d ROBOT_TO_CAMERA_3 =
      new Transform3d(camera_y - 0.0889, 0, camera_h - 0.1016, new Rotation3d(0, 0, Math.PI));

  // Max pitch/roll allowed before ignoring vision poses (bump handling)
  public static final double MAX_PITCH_ROLL_DEGREES = 3.0;

  // Field dimensions and safeguards (meters)
  public static final double FIELD_LENGTH_METERS = 16.54;
  public static final double FIELD_WIDTH_METERS = 9.14;

  // Prevent odometry from jumping extremely far based on 1 erroneous vision frame
  public static final double MAX_POSE_DIFFERENCE_METERS = 2.0;

  // Reject single-tag estimates with ambiguity above this threshold
  public static final double MAX_AMBIGUITY = 0.2;
}
