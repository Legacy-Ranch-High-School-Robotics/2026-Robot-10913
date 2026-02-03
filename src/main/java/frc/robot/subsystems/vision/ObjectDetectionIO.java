package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public double[] timestamps = new double[0];
    public String[] classNames = new String[0];
    public double[] confidences = new double[0];
    public double[] distances = new double[0];
    public double[] yawAngles = new double[0]; // Radians
  }

  default void updateInputs(ObjectDetectionIOInputs inputs) {}

  public static class DetectedObject {
    public final String className;
    public final double confidence;
    public final double distance;
    public final Rotation2d yaw;

    public DetectedObject(String className, double confidence, double distance, Rotation2d yaw) {
      this.className = className;
      this.confidence = confidence;
      this.distance = distance;
      this.yaw = yaw;
    }
  }
}
