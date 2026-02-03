package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;

public class VisionIOPhoton implements VisionIO {
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;

  public VisionIOPhoton(String cameraName, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    var results = camera.getAllUnreadResults();

    // 1. Create temp lists
    List<Pose3d> poses = new ArrayList<>();
    List<Double> timestamps = new ArrayList<>();
    List<Integer> tagCounts = new ArrayList<>();
    List<Double> stdDevs = new ArrayList<>();

    // 2. Loop through every result (Batched Data)
    for (var result : results) {
      var multiTagResult = result.getMultiTagResult();
      if (multiTagResult.isPresent()) {
        var best = multiTagResult.get().estimatedPose.best;

        // Transform FieldToCamera -> RobotPose
        Pose3d cameraPose = new Pose3d(best.getTranslation(), best.getRotation());
        Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());

        // Add to lists
        poses.add(robotPose);
        timestamps.add(result.getTimestampSeconds());
        tagCounts.add(multiTagResult.get().fiducialIDsUsed.size());

        // Add StdDevs (Example: 0.5m default, or calculate based on tag dist)
        stdDevs.add(0.5); // X
        stdDevs.add(0.5); // Y
        stdDevs.add(1.0); // Rotation (Radians)
      }
    }

    // 3. Convert to Arrays for AutoLog
    inputs.estimatedPoses = poses.toArray(new Pose3d[0]);
    inputs.timestamps = timestamps.stream().mapToDouble(d -> d).toArray();
    inputs.tagCounts = tagCounts.stream().mapToInt(i -> i).toArray();
    inputs.visionStdDevs = stdDevs.stream().mapToDouble(d -> d).toArray();
  }
}
