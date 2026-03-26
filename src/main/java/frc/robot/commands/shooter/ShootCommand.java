package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShootCommand extends Command {
  private final Shooter shooter;
  private final Hopper hopper;
  private final double shooterRPM;
  private final double hopperRPM;

  public ShootCommand(Shooter shooter, Hopper hopper, double shooterRPM, double hopperRPM) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.shooterRPM = shooterRPM;
    this.hopperRPM = hopperRPM;
    addRequirements(shooter, hopper);
  }

  public ShootCommand(Shooter shooter, Hopper hopper) {
    this(shooter, hopper, ShooterConstants.shooterRPM, HopperConstants.hopperFeedRPM);
  }

  @Override
  public void initialize() {
    shooter.setVelocity(shooterRPM);
    hopper.setVelocity(hopperRPM);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    hopper.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
