package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANFuelSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterSequence extends SequentialCommandGroup {
  /** Creates a new LaunchSequence. */
  public ShooterSequence(CANFuelSubsystem fuelSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        new Launch(fuelSubsystem));
  }
}
