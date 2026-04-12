package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.telemetry.ElasticTelemetry;

public final class RobotCommands {

  private RobotCommands() {}

  // ==================== INTAKE ====================

  public static Command deployIntake(Intake intake) {
    return Commands.run(intake::liftDeploy, intake)
        .until(intake::isLiftDeployed)
        .finallyDo(() -> intake.liftStop());
  }

  public static Command retractIntake(Intake intake) {
    return Commands.run(intake::liftRetract, intake)
        .until(intake::isLiftRetracted)
        .finallyDo(() -> intake.liftStop());
  }

  public static Command intakeRollers(Intake intake) {
    return Commands.run(intake::intake, intake).finallyDo(() -> intake.stop());
  }

  public static Command outtakeRollers(Intake intake) {
    return Commands.run(intake::outtake, intake).finallyDo(() -> intake.stop());
  }

  public static Command stopIntake(Intake intake) {
    return Commands.runOnce(intake::stop, intake);
  }

  // ==================== SHOOTER ====================

  public static Command spinUpShooter(Shooter shooter, double rpm) {
    return Commands.runOnce(() -> shooter.setVelocity(rpm), shooter)
        .andThen(Commands.waitUntil(shooter::atTargetVelocity).withTimeout(5.0));
  }

  public static Command spinUpShooter(Shooter shooter) {
    return spinUpShooter(shooter, ShooterConstants.shooterRPM);
  }

  public static Command spinUpAndHold(Shooter shooter) {
    return Commands.run(
            () ->
                shooter.setVelocity(
                    ElasticTelemetry.getNumber("Shooter/Target RPM", ShooterConstants.shooterRPM)),
            shooter)
        .finallyDo(() -> shooter.stop());
  }

  public static Command stopShooter(Shooter shooter) {
    return Commands.runOnce(shooter::stop, shooter);
  }

  // ==================== COMBINED ====================

  public static Command shoot(Shooter shooter, Hopper hopper) {
    return Commands.run(
            () -> {
              shooter.setVelocity(ShooterConstants.shooterRPM);
              if (shooter.atTargetVelocity()) {
                hopper.setVelocity(HopperConstants.hopperFeedRPM);
              } else {
                hopper.stop();
              }
            },
            shooter,
            hopper)
        .finallyDo(
            () -> {
              shooter.stop();
              hopper.stop();
            });
  }

  public static Command launch(Shooter shooter, Hopper hopper) {
    return Commands.run(
            () -> {
              shooter.setVelocity(
                  ElasticTelemetry.getNumber("Shooter/Target RPM", ShooterConstants.shooterRPM));
              if (shooter.atTargetVelocity()) {
                hopper.setVelocity(HopperConstants.hopperFeedRPM);
              } else {
                hopper.stop();
              }
            },
            shooter,
            hopper)
        .finallyDo(
            () -> {
              shooter.stop();
              hopper.stop();
            });
  }

  public static Command eject(Intake intake, Hopper hopper, Shooter shooter) {
    return Commands.run(
            () -> {
              intake.intake();
              hopper.eject();
              shooter.eject();
            },
            intake,
            hopper,
            shooter)
        .finallyDo(
            () -> {
              intake.stop();
              hopper.stop();
              shooter.stop();
            });
  }

  // ==================== SHOOT ON MOVE ====================

  public static class ShootOnMove extends edu.wpi.first.wpilibj2.command.Command {
    private final Shooter m_shooter;
    private final Hopper m_hopper;
    private final DriveSubsystem m_drive;

    private static final double VELOCITY_COMPENSATION_FACTOR = 100.0;

    public ShootOnMove(Shooter shooter, Hopper hopper, DriveSubsystem drive) {
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
      ChassisSpeeds velocity = m_drive.getChassisSpeeds();
      double baseRPM = ShooterConstants.distanceToRpmMap.get(distance);

      var targetAngle = m_drive.getTargetAngleToHub();
      double velocityTowardTarget =
          velocity.vxMetersPerSecond * targetAngle.getCos()
              + velocity.vyMetersPerSecond * targetAngle.getSin();

      double compensatedRPM =
          Math.max(
              2000.0,
              Math.min(6000.0, baseRPM - velocityTowardTarget * VELOCITY_COMPENSATION_FACTOR));

      ElasticTelemetry.setNumber("ShootOnMove/Distance", distance);
      ElasticTelemetry.setNumber("ShootOnMove/BaseRPM", baseRPM);
      ElasticTelemetry.setNumber("ShootOnMove/VelocityTowardTarget", velocityTowardTarget);
      ElasticTelemetry.setNumber("ShootOnMove/CompensatedRPM", compensatedRPM);
      ElasticTelemetry.setNumber(
          "ShootOnMove/AngleError", m_drive.getAngleErrorToHub().getDegrees());

      m_shooter.setVelocity(compensatedRPM);

      if (m_shooter.atTargetVelocity() && isAimedAtHub()) {
        m_hopper.setVelocity(HopperConstants.hopperFeedRPM);
        ElasticTelemetry.setBoolean("ShootOnMove/Feeding", true);
      } else {
        m_hopper.stop();
        ElasticTelemetry.setBoolean("ShootOnMove/Feeding", false);
      }
    }

    @Override
    public void end(boolean interrupted) {
      m_shooter.stop();
      m_hopper.stop();
      m_drive.setTrackingHub(false);
      ElasticTelemetry.setBoolean("ShootOnMove/Feeding", false);
    }

    @Override
    public boolean isFinished() {
      return false;
    }

    private boolean isAimedAtHub() {
      return Math.abs(m_drive.getAngleErrorToHub().getDegrees()) < 5.0;
    }
  }
}
