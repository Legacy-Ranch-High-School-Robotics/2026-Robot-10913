package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends Command {
  private final Intake intake;

  public IntakeCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.intake();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
