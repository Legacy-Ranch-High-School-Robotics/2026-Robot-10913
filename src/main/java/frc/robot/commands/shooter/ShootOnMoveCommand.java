package frc.robot.commands.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.telemetry.ElasticTelemetry;

/**
 * Command that shoots while moving by compensating for robot velocity.
 * 
 * <p>This command:
 * - Calculates base shooter RPM from distance to hub using ShooterConstants.distanceToRpmMap
 * - Measures robot velocity component toward target
 * - Adds velocity compensation to base RPM (helps maintain shot trajectory while moving)
 * - Automatically feeds hopper when shooter is at target velocity and robot is aimed at hub
 * - Enables hub tracking so driver can aim while command runs
 * 
 * <p>Velocity Compensation Formula:
 * compensatedRPM = baseRPM + (velocityTowardTarget * VELOCITY_COMPENSATION_FACTOR)
 * 
 * <p>Usage: Operator triggers this command, driver aims robot at hub, 
 * command handles shooter RPM and hopper feeding automatically.
 */
public class ShootOnMoveCommand extends Command {
  private final Shooter m_shooter;
  private final Hopper m_hopper;
  private final DriveSubsystem m_drive;

  // Velocity compensation factor - how much RPM to add per m/s of robot velocity toward target
  private static final double VELOCITY_COMPENSATION_FACTOR = 100.0;

  // Clamp RPM to safe operating range
  private static final double MIN_SAFE_RPM = 2000.0;
  private static final double MAX_SAFE_RPM = 6000.0;

  public ShootOnMoveCommand(Shooter shooter, Hopper hopper, DriveSubsystem drive) {
    m_shooter = shooter;
    m_hopper = hopper;
    m_drive = drive;
    addRequirements(shooter, hopper);
  }

  @Override
  public void initialize() {
    // Enable hub tracking so driver can aim while this command runs
    m_drive.setTrackingHub(true);
    ElasticTelemetry.setBoolean("ShootOnMove/Active", true);
  }

  @Override
  public void execute() {
    // Get current distance to hub and robot velocity
    double distance = m_drive.getDistanceToHub();
    ChassisSpeeds robotVelocity = m_drive.getChassisSpeeds();

    // Calculate base shooter RPM from distance using the proper interpolation map
    double baseRPM = ShooterConstants.distanceToRpmMap.get(distance);

    // Calculate robot velocity component toward the hub
    var targetAngle = m_drive.getTargetAngleToHub();
    double vx = robotVelocity.vxMetersPerSecond;
    double vy = robotVelocity.vyMetersPerSecond;
    double targetCos = targetAngle.getCos();
    double targetSin = targetAngle.getSin();
    double velocityTowardTarget = vx * targetCos + vy * targetSin;

    // Apply velocity compensation and clamp to safe range
    double compensatedRPM = baseRPM + (velocityTowardTarget * VELOCITY_COMPENSATION_FACTOR);
    compensatedRPM = Math.max(MIN_SAFE_RPM, Math.min(MAX_SAFE_RPM, compensatedRPM));

    // Publish telemetry for debugging
    ElasticTelemetry.setNumber("ShootOnMove/Distance", distance);
    ElasticTelemetry.setNumber("ShootOnMove/BaseRPM", baseRPM);
    ElasticTelemetry.setNumber("ShootOnMove/VelocityTowardTarget", velocityTowardTarget);
    ElasticTelemetry.setNumber("ShootOnMove/CompensatedRPM", compensatedRPM);
    ElasticTelemetry.setNumber("ShootOnMove/AngleError", m_drive.getAngleErrorToHub().getDegrees());

    // Set shooter to compensated RPM
    m_shooter.setVelocity(compensatedRPM);

    // Feed hopper only when shooter is ready and robot is aimed
    if (m_shooter.atTargetVelocity() && isAimedAtHub()) {
      m_hopper.setVelocity(frc.robot.subsystems.hopper.HopperConstants.hopperFeedRPM);
      ElasticTelemetry.setBoolean("ShootOnMove/Feeding", true);
    } else {
      m_hopper.stop();
      ElasticTelemetry.setBoolean("ShootOnMove/Feeding", false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Clean up: stop all mechanisms and disable hub tracking
    m_shooter.stop();
    m_hopper.stop();
    m_drive.setTrackingHub(false);
    ElasticTelemetry.setBoolean("ShootOnMove/Active", false);
    ElasticTelemetry.setBoolean("ShootOnMove/Feeding", false);
  }

  @Override
  public boolean isFinished() {
    // This command runs until manually cancelled
    return false;
  }

  /**
   * Check if robot is aimed within acceptable angle of hub.
   * Uses 5-degree tolerance for consistent shots.
   * 
   * @return true if robot is aimed at hub within tolerance
   */
  private boolean isAimedAtHub() {
    return Math.abs(m_drive.getAngleErrorToHub().getDegrees()) < 5.0;
  }
}
