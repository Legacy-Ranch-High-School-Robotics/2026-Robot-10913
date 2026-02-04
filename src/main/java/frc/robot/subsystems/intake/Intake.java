package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /** Run the intake at the specified voltage. */
  public void runIntake() {
    io.setRunning(true);
  }

  /** Stop the intake. */
  public void stop() {
    io.setRunning(false);
  }

  /** Command to run the intake. */
  public Command runCommand() {
    return this.startEnd(this::runIntake, this::stop);
  }

  /** Command to stop the intake. */
  public Command stopCommand() {
    return this.runOnce(this::stop);
  }
}
