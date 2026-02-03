package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // Camera Names
  public static final String frontTagCamName = "frontTag";
  public static final String backTagCamName = "backTag";
  public static final String frontMLCamName = "frontML";

  // Camera Transforms (Robot Center to Camera)
  // TODO: Update these with real physical measurements
  public static final Transform3d frontTagCamTransform =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(10)),
          new Rotation3d(0, 0, 0));

  public static final Transform3d backTagCamTransform =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-10), 0, Units.inchesToMeters(10)),
          new Rotation3d(0, 0, Math.PI));

  public static final Transform3d frontMLCamTransform =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(20)),
          new Rotation3d(0, Units.degreesToRadians(-15), 0));

  // Vision Standard Deviations
  // Trust vision less than odometry usually, but it depends on the strategy
  public static final double linearStdDev = 0.5; // Meters
  public static final double angularStdDev = 1.0; // Radians

  // Minimal tags visible for "good" read
  public static final int minTags = 2;

  // Field Limits for Filtering
  public static final double fieldLength = 17.55; // Meters
  public static final double fieldWidth = 8.05; // Meters
  public static final double fieldBorderMargin = 0.5; // Meters
  public static final double zMargin = 0.75; // Meters (Max height for valid robot pose)
}
