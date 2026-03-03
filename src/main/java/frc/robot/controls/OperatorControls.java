package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;

public class OperatorControls {

  public static void configure(int port, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    // Intake controls
    controller.start().onTrue(superstructure.rezeroIntakePivotCommand().ignoringDisable(true));

    controller.rightBumper()
        .whileTrue(superstructure.setIntakeDeployAndRoll().withName("OperatorControls.intakeDeployed"));

    controller.y().whileTrue(superstructure.intakeCommand().withName("OperatorControls.intake"));

    controller.a().whileTrue(superstructure.ejectCommand().withName("OperatorControls.eject"));
  }
}
