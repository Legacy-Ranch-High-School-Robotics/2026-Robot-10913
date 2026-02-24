package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Superstructure coordinates the intake subsystem for unified control.
 */
public class Superstructure extends SubsystemBase {

  public final IntakeSubsystem intake;

  public Superstructure(IntakeSubsystem intake) {
    this.intake = intake;
  }

  /**
   * Command to run the intake while held.
   */
  public Command intakeCommand() {
    return intake.intakeCommand().withName("Superstructure.intake");
  }

  /**
   * Command to eject while held.
   */
  public Command ejectCommand() {
    return intake.ejectCommand().withName("Superstructure.eject");
  }

  /**
   * Command to set the intake pivot angle.
   */
  public Command setIntakePivotAngle(Angle angle) {
    return intake.setPivotAngle(angle).withName("Superstructure.setIntakePivotAngle");
  }

  /**
   * Command to deploy intake and run wheels.
   */
  public Command setIntakeDeployAndRoll() {
    return intake.deployAndRollCommand().withName("Superstructure.setIntakeDeployAndRoll");
  }

  /**
   * Command to back feed with intake.
   */
  public Command backFeedIntakeCommand() {
    return intake.backFeedAndRollCommand().withName("Superstructure.backFeedIntake");
  }

  /**
   * Command to rezero intake pivot.
   */
  public Command rezeroIntakePivotCommand() {
    return intake.rezero().withName("Superstructure.rezeroIntakePivot");
  }

  @Override
  public void periodic() {
    // Superstructure handles intake coordination
  }
}
