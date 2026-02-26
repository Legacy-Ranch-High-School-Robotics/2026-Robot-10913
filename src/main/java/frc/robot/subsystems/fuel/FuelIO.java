package frc.robot.subsystems.fuel;

import org.littletonrobotics.junction.AutoLog;

public interface FuelIO {

  @AutoLog
  public static class FuelIOInputs {
    // --- Left Launcher / Intake ---
    public double leftLauncherPositionRad = 0.0;
    public double leftLauncherVelocityRadPerSec = 0.0;
    public double leftLauncherAppliedVolts = 0.0;
    public double leftLauncherCurrentAmps = 0.0;

    // --- Right Launcher / Intake ---
    public double rightLauncherPositionRad = 0.0;
    public double rightLauncherVelocityRadPerSec = 0.0;
    public double rightLauncherAppliedVolts = 0.0;
    public double rightLauncherCurrentAmps = 0.0;

    // --- Indexer ---
    public double indexerPositionRad = 0.0;
    public double indexerVelocityRadPerSec = 0.0;
    public double indexerAppliedVolts = 0.0;
    public double indexerCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FuelIOInputs inputs) {}

  // Set the voltage of the left intake/launcher roller
  public default void setLeftIntakeLauncherRoller(double power) {}

  // Set the voltage of the right intake/launcher roller
  public default void setRightIntakeLauncherRoller(double power) {}

  // Set the voltage of the feeder roller
  public default void setFeederRoller(double power) {}

  // Stop the rollers
  public default void stop() {}
}
