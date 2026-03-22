package frc.robot.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSim {
  private final VisionSystemSim m_visionSystemSim;

  /**
   * Creates a VisionSim that wraps the SAME PhotonCamera instances used by the Vision subsystem.
   * This is critical: PhotonCameraSim must wrap the exact same PhotonCamera object that the
   * subsystem reads from, otherwise the simulated results are lost.
   */
  public VisionSim(
      AprilTagFieldLayout fieldLayout,
      PhotonCamera camera1,
      PhotonCamera camera2,
      PhotonCamera camera3) {
    m_visionSystemSim = new VisionSystemSim("main");

    if (fieldLayout != null) {
      m_visionSystemSim.addAprilTags(fieldLayout);
    }

    SimCameraProperties properties = SimConstants.createCameraProperties();

    PhotonCameraSim camera1Sim = new PhotonCameraSim(camera1, properties);
    PhotonCameraSim camera2Sim = new PhotonCameraSim(camera2, properties);
    PhotonCameraSim camera3Sim = new PhotonCameraSim(camera3, properties);

    m_visionSystemSim.addCamera(camera1Sim, VisionConstants.ROBOT_TO_CAMERA_1);
    m_visionSystemSim.addCamera(camera2Sim, VisionConstants.ROBOT_TO_CAMERA_2);
    m_visionSystemSim.addCamera(camera3Sim, VisionConstants.ROBOT_TO_CAMERA_3);
  }

  public void update(Pose2d actualRobotPose) {
    m_visionSystemSim.update(actualRobotPose);
  }
}
