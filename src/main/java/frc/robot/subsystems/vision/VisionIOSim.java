package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {
  private final PhotonCameraSim cameraSim;
  private final PhotonPoseEstimator coprocessorEstimator;
  private final VisionSystemSim visionSim;
  private final Transform3d robotToCamera;

  public VisionIOSim(
      AprilTagFieldLayout fieldLayout,
      Transform3d robotToCamera,
      String cameraName,
      VisionSystemSim sharedVisionSim) {
    this.robotToCamera = robotToCamera;
    visionSim = sharedVisionSim;

    // 1. Setup Camera Properties (Simulating hardware)
    // The PhotonCamera used in the real robot code.
    PhotonCamera camera = new PhotonCamera(cameraName);
    SimCameraProperties props = SimCameraProperties.LL2_960_720();
    cameraSim = new PhotonCameraSim(camera, props);

    // 2. Add camera to the shared Vision System
    visionSim.addCamera(cameraSim, robotToCamera);

    // 3. Setup the Virtual Coprocessor Logic
    // We use PhotonPoseEstimator here to MIMIC what the real coprocessor does.
    coprocessorEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // A. Generate the raw "image" (list of targets) from physics
    var results = cameraSim.getCamera().getAllUnreadResults();

    // 1. Create temp lists
    List<Pose3d> poses = new ArrayList<>();
    List<Double> timestamps = new ArrayList<>();
    List<Integer> tagCounts = new ArrayList<>();
    List<Double> stdDevs = new ArrayList<>();

    for (var result : results) {

      // B. Feed raw targets into the Virtual Coprocessor
      // var estimatedPoseOpt = coprocessorEstimator.estimateCoprocMultiTagPose()
      // (rawResults);

      Optional<EstimatedRobotPose> visionEst =
          coprocessorEstimator.estimateCoprocMultiTagPose(result);
      if (!visionEst.isPresent()) {
        visionEst = coprocessorEstimator.estimateLowestAmbiguityPose(result);
      }

      // C. Log the result to AdvantageKit
      if (visionEst.isPresent()) {
        var pose = visionEst.get().estimatedPose;

        // Pose is already the Robot Pose because we passed the robotToCamera transform to the
        // estimator
        Pose3d robotPose = pose;

        // Add to lists
        poses.add(robotPose);
        timestamps.add(result.getTimestampSeconds());
        tagCounts.add(visionEst.get().targetsUsed.size());

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

  @Override
  public void updateSimPose(Pose3d trueRobotPose) {
    // This is handled by the shared VisionSystemSim update in RobotContainer
    // But we define it here if we want per-camera updates (logic depends on how
    // VisionSystemSim is
    // updated)
    // Actually, VisionSystemSim updates ALL cameras when updated.
    // So this method might be redundant if we update the shared VisionSystemSim
    // centrally.
    // However, for interface compliance, we can leave it empty or use it if we had
    // separate
    // systems.
  }
}
