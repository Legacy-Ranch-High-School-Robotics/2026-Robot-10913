package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Generates a Vision Pose (Camera Illusion) by looking at the "Actual Pose" and outputting
 * simulated AprilTag data to the student code.
 */
public class VisionSim {

  public VisionSim() {
    // TODO: Initialize a PhotonCameraSim (or equivalent WPILib vision sim)
    // loaded with the 2026 REBUILT AprilTag field layout.
  }

  /**
   * Feed the "Actual Pose" into the camera simulator so it generates realistic targets and noise.
   *
   * @param actualPose The ground truth pose from maple-sim.
   */
  public void update(Pose2d actualPose) {
    // TODO: Feed actualPose into the camera simulator
  }
}
