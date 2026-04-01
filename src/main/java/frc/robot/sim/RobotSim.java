package frc.robot.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

/**
 * The coordinator master class for Simulation. Ensures the strict execution order: 1)
 * DriveSim.update() 2) VisionSim.update(DriveSim.getActualPose()) 3) Mechanism sims update 4)
 * Publish Actual Pose to SmartDashboard/AdvantageScope
 */
public class RobotSim {

  private final DriveSubsystem m_driveSubsystem;
  private final VisionSim m_visionSim;
  private final ShooterSim m_shooterSim;
  private final HopperSim m_hopperSim;
  private final IntakeSim m_intakeSim;

  public RobotSim(
      DriveSubsystem driveSubsystem,
      AprilTagFieldLayout fieldLayout,
      Vision vision,
      Shooter shooter,
      Hopper hopper,
      Intake intake) {
    m_driveSubsystem = driveSubsystem;
    m_visionSim =
        new VisionSim(
            fieldLayout,
            vision.getCamera1(),
            vision.getCamera2(),
            vision.getCamera3(),
            () -> m_driveSubsystem.getDriveSim().getActualPose());
    m_shooterSim = new ShooterSim(shooter);
    m_hopperSim = new HopperSim(hopper);
    m_intakeSim = new IntakeSim(intake);
  }

  /** Call this in robotPeriodic() or simulationPeriodic(). */
  public void simulatorPeriodic() {
    DriveSim driveSim = m_driveSubsystem.getDriveSim();

    if (driveSim != null) {
      // 1) Step drivetrain physics (also injects gyro values)
      driveSim.update();

      // 2) Publish the ground truth pose for visual comparison
      m_driveSubsystem.getField().getObject("Actual Robot").setPose(driveSim.getActualPose());
    }

    // 4) Step mechanism simulations
    m_shooterSim.update();
    m_hopperSim.update();
    m_intakeSim.update();
  }
}
