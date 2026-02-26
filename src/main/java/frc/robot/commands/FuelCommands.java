package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.fuel.Fuel;

public class FuelCommands {

  // Private constructor prevents this utility class from being instantiated
  private FuelCommands() {}

  /** Fires the game piece by running both the launchers and the feeder. */
  public static Command launch(Fuel fuel) {
    return Commands.runEnd(
        () -> {
          fuel.setIntakeLauncherRoller(12.0);
          fuel.setFeederRoller(12.0);
        },
        () -> fuel.stop(),
        fuel);
  }

  // TODO: Add commands for intaking, ejecting, and spinning up
}
