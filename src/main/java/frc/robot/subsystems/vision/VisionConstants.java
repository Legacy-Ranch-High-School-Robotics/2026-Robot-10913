package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public final class VisionConstants {
  public static final String CAMERA_1_NAME = "Arducam3-front-10913";
  public static final String CAMERA_2_NAME = "Arducam2-back-10913";
  public static final String CAMERA_3_NAME = "Arducam2-side-10913";

  // Robot to camera transforms
  public static final Transform3d ROBOT_TO_CAMERA_1 =
      new Transform3d(0.5, 0.0, 0.5, new Rotation3d(0, 0, 0));
  public static final Transform3d ROBOT_TO_CAMERA_2 =
      new Transform3d(-0.5, 0.2, 0.5, new Rotation3d(0, 0, Math.PI));
  public static final Transform3d ROBOT_TO_CAMERA_3 =
      new Transform3d(-0.5, -0.2, 0.5, new Rotation3d(0, 0, Math.PI));

  // Max pitch/roll allowed before ignoring vision poses (bump handling)
  public static final double MAX_PITCH_ROLL_DEGREES = 3.0;

  // Field dimensions and safeguards (meters)
  public static final double FIELD_LENGTH_METERS = 16.54;
  public static final double FIELD_WIDTH_METERS = 9.14;

  // Prevent odometry from jumping extremely far based on 1 erroneous vision frame
  public static final double MAX_POSE_DIFFERENCE_METERS = 5.0;
}
