package frc.robot.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;

/**
 * The coordinator master class for Simulation. Ensures the strict execution order: 1)
 * DriveSim.update() 2) VisionSim.update(DriveSim.getActualPose()) 3) Publish Actual Pose to
 * SmartDashboard/AdvantageScope
 */
public class RobotSim {

  private final DriveSubsystem m_driveSubsystem;
  private final VisionSim m_visionSim;

  public RobotSim(DriveSubsystem driveSubsystem, AprilTagFieldLayout fieldLayout, Vision vision) {
    m_driveSubsystem = driveSubsystem;
    m_visionSim =
        new VisionSim(fieldLayout, vision.getCamera1(), vision.getCamera2(), vision.getCamera3());
  }

  /** Call this in robotPeriodic() or simulationPeriodic(). */
  public void simulatorPeriodic() {
    DriveSim driveSim = m_driveSubsystem.getDriveSim();

    if (driveSim != null) {
      // 1) Step drivetrain physics
      driveSim.update();

      // 2) Step vision, providing the new ground truth pose
      m_visionSim.update(driveSim.getActualPose());

      // 3) Publish the ground truth pose for visual comparison
      m_driveSubsystem.getField().getObject("Actual Robot").setPose(driveSim.getActualPose());
    }
  }
}
