package frc.robot.sim;

import frc.robot.subsystems.DriveSubsystem;

/**
 * The coordinator master class for Simulation. Ensures the strict execution order: 1)
 * DriveSim.update() 2) VisionSim.update(DriveSim.getActualPose()) 3) Publish Actual Pose to
 * SmartDashboard/AdvantageScope
 */
public class RobotSim {

  private final DriveSubsystem m_driveSubsystem;
  private final VisionSim m_visionSim;

  public RobotSim(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_visionSim = new VisionSim();
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
