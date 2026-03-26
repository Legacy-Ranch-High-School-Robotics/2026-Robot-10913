package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class SpinUpShooter extends Command {
  private final Shooter shooter;
  private final double targetRPM;

  public SpinUpShooter(Shooter shooter, double targetRPM) {
    this.shooter = shooter;
    this.targetRPM = targetRPM;
    addRequirements(shooter);
  }

  public SpinUpShooter(Shooter shooter) {
    this(shooter, ShooterConstants.shooterRPM);
  }

  @Override
  public void initialize() {
    shooter.setVelocity(targetRPM);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return shooter.atTargetVelocity();
  }
}
