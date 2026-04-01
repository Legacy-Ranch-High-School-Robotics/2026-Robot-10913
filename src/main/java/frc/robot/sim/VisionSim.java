package frc.robot.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSim {
  private final VisionSystemSim m_visionSystemSim;

  // Create a Notifier for the background thread
  private final Notifier m_simNotifier;

  /**
   * We add a Supplier<Pose2d> to the constructor so the background thread can constantly poll the
   * robot's current pose without needing it passed in manually.
   */
  public VisionSim(
      AprilTagFieldLayout fieldLayout,
      PhotonCamera camera1,
      PhotonCamera camera2,
      PhotonCamera camera3,
      Supplier<Pose2d> robotPoseSupplier) {

    m_visionSystemSim = new VisionSystemSim("main");

    if (fieldLayout != null) {
      m_visionSystemSim.addAprilTags(fieldLayout);
    }

    // Keep your existing setup...
    SimCameraProperties properties = SimConstants.createCameraProperties();

    PhotonCameraSim camera1Sim = new PhotonCameraSim(camera1, properties);
    PhotonCameraSim camera2Sim = new PhotonCameraSim(camera2, properties);
    PhotonCameraSim camera3Sim = new PhotonCameraSim(camera3, properties);

    m_visionSystemSim.addCamera(camera1Sim, VisionConstants.ROBOT_TO_CAMERA_1);
    m_visionSystemSim.addCamera(camera2Sim, VisionConstants.ROBOT_TO_CAMERA_2);
    m_visionSystemSim.addCamera(camera3Sim, VisionConstants.ROBOT_TO_CAMERA_3);

    // Define the background runner
    m_simNotifier =
        new Notifier(
            () -> {
              // Grab the latest pose and update the heavy 3D math off the main thread
              m_visionSystemSim.update(robotPoseSupplier.get());
            });

    // Start the background thread at 50Hz (every 0.02 seconds)
    // If it takes 80ms to run, it won't stall the main robot loop!
    m_simNotifier.startPeriodic(0.02);
  }
}
