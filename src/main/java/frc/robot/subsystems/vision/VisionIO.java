package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {

    // Contains results
    public int size() {
      return estimatedPoses.length;
    }

    // A list of all poses calculated since the last loop
    public Pose3d[] estimatedPoses = new Pose3d[0];

    // The timestamp for EACH pose in the list above
    public double[] timestamps = new double[0];

    // The number of tags seen for EACH pose (used for filtering)
    // Index 0 corresponds to estimatedPoses[0], etc.
    public int[] tagCounts = new int[0];

    // Flattened standard deviations: [x1, y1, r1, x2, y2, r2, ...]
    public double[] visionStdDevs = new double[0];
  }

  default void updateInputs(VisionIOInputs inputs) {}

  /** Updates the Sim with the true robot pose. */
  default void updateSimPose(Pose3d trueRobotPose) {}
}
