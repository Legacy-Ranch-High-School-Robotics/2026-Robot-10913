package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean isRunning = false;
    public int gamePiecesCount = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the intake at the specified voltage. */
  public default void setRunning(boolean runIntake) {}

  public default int getFuelCount() {
    return 0;
  }
}
