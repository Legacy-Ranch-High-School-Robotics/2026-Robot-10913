package frc.robot.subsystems.fuel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Fuel extends SubsystemBase {
  private final FuelIO io;
  private final FuelIOInputsAutoLogged inputs = new FuelIOInputsAutoLogged();

  // Dependency injection: the robot container passes the hardware in
  public Fuel(FuelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // 1. Read from the hardware
    io.updateInputs(inputs);
    // 2. Log the data to AdvantageKit
    Logger.processInputs("Fuel", inputs);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double power) {
    io.setLeftIntakeLauncherRoller(power);
    io.setRightIntakeLauncherRoller(power);
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double power) {
    // TODO: update this code to use the IO layer to set the feeder roller
  }

  // A method to stop the rollers
  public void stop() {
    // TODO: update this code to use the IO layer to stop all rollers
  }
}
