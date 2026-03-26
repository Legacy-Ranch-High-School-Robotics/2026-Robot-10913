package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RetractIntake extends Command {
  private final Intake intake;

  public RetractIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.liftRetract();
  }

  @Override
  public void end(boolean interrupted) {
    intake.liftStop();
  }

  @Override
  public boolean isFinished() {
    return intake.isLiftRetracted();
  }
}
