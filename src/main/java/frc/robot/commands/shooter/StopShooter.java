package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.Shooter;

public class StopShooter extends InstantCommand {
  public StopShooter(Shooter shooter) {
    super(() -> shooter.stop(), shooter);
  }
}
