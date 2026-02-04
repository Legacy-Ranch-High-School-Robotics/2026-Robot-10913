package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.intakeLengthExtended;
import static frc.robot.subsystems.intake.IntakeConstants.intakeStorageCapacity;
import static frc.robot.subsystems.intake.IntakeConstants.intakeWidth;

import edu.wpi.first.units.Units;
import frc.robot.Constants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {
  private final IntakeSimulation intakeSim;

  public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
    // Game Piece Type: REBUILT_GAME_PIECE
    // Capacity: 10
    intakeSim =
        IntakeSimulation.OverTheBumperIntake(
            Constants.REBUILT_GAME_PIECE,
            driveTrain,
            Units.Meters.of(intakeWidth),
            Units.Meters.of(intakeLengthExtended),
            IntakeSide.FRONT,
            intakeStorageCapacity);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isRunning = intakeSim.isRunning();
    inputs.gamePiecesCount = intakeSim.getGamePiecesAmount();
  }

  @Override // Defined by IntakeIO
  public void setRunning(boolean runIntake) {
    if (runIntake)
      intakeSim.startIntake(); // Extends the intake out from the chassis frame and starts detecting
    // contacts with game pieces
    else intakeSim.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece
    // collection
  }
}
