package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.telemetry.ElasticTelemetry;

public class AutoShootCommand extends Command {
  private final Shooter m_shooter;
  private final Hopper m_hopper;
  private final DriveSubsystem m_drive;

  private static final double[][] SHOOTER_MAP = {
    {1.0, 3000.0},
    {2.0, 3500.0},
    {3.0, 4000.0},
    {4.0, 4500.0},
    {5.0, 5000.0},
    {6.0, 5500.0}
  };

  public AutoShootCommand(Shooter shooter, Hopper hopper, DriveSubsystem drive) {
    m_shooter = shooter;
    m_hopper = hopper;
    m_drive = drive;
    addRequirements(shooter, hopper);
  }

  @Override
  public void initialize() {
    m_drive.setTrackingHub(true);
  }

  @Override
  public void execute() {
    double distance = m_drive.getDistanceToHub();
    double targetRPM = interpolateShooterRPM(distance);

    ElasticTelemetry.setNumber("AutoShoot/Distance", distance);
    ElasticTelemetry.setNumber("AutoShoot/CalculatedRPM", targetRPM);
    ElasticTelemetry.setNumber(
        "AutoShoot/AngleError", m_drive.getAngleErrorToHub().getDegrees());

    m_shooter.setVelocity(targetRPM);

    if (m_shooter.atTargetVelocity() && isAimedAtHub()) {
      m_hopper.setVelocity(frc.robot.subsystems.hopper.HopperConstants.hopperFeedRPM);
      ElasticTelemetry.setBoolean("AutoShoot/Feeding", true);
    } else {
      m_hopper.stop();
      ElasticTelemetry.setBoolean("AutoShoot/Feeding", false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_hopper.stop();
    m_drive.setTrackingHub(false);
    ElasticTelemetry.setBoolean("AutoShoot/Feeding", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double interpolateShooterRPM(double distance) {
    if (distance <= SHOOTER_MAP[0][0]) {
      return SHOOTER_MAP[0][1];
    }
    if (distance >= SHOOTER_MAP[SHOOTER_MAP.length - 1][0]) {
      return SHOOTER_MAP[SHOOTER_MAP.length - 1][1];
    }

    for (int i = 0; i < SHOOTER_MAP.length - 1; i++) {
      if (distance >= SHOOTER_MAP[i][0] && distance <= SHOOTER_MAP[i + 1][0]) {
        double x0 = SHOOTER_MAP[i][0];
        double y0 = SHOOTER_MAP[i][1];
        double x1 = SHOOTER_MAP[i + 1][0];
        double y1 = SHOOTER_MAP[i + 1][1];
        return y0 + (distance - x0) * (y1 - y0) / (x1 - x0);
      }
    }

    return ShooterConstants.shooterRPM;
  }

  private boolean isAimedAtHub() {
    return Math.abs(m_drive.getAngleErrorToHub().getDegrees()) < 3.0;
  }
}
