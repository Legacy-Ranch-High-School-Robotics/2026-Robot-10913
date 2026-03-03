package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;

public class DriverControls {

  public static void configure(int port, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    // --- Intake Controls Only ---

    // Right bumper: deploy intake + run roller while held
    controller.rightBumper()
        .whileTrue(
            superstructure.setIntakeDeployAndRoll()
                .withName("DriverControls.IntakeDeployAndRoll")
        );

    // Left bumper: back-feed intake while held
    // Use the existing Superstructure.backFeedIntakeCommand which already
    // stops the intake when released (via its finallyDo in the IntakeSubsystem).
    controller.leftBumper()
        .whileTrue(
            superstructure.backFeedIntakeCommand()
                .withName("DriverControls.FeedAll")
        );
  }
}