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

    // Left bumper: feed all (includes intake backfeeding)
    controller.leftBumper()
        .whileTrue(
            superstructure.feedAllCommand()
                .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule())
        );
  }
}