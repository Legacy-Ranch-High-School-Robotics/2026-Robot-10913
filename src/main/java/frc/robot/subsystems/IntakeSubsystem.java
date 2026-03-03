package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private static final double INTAKE_SPEED = 0.8;
  private static final double INTAKE_MAX_RPM = 1000;

  // Intake roller motor
  private SparkMax intakeRollerMotor;
  private RelativeEncoder intakeRollerEncoder;

  // Intake pivot motor
  private SparkMax intakePivotMotor;
  private RelativeEncoder intakePivotEncoder;
  private SparkClosedLoopController intakePivotClosedLoopController;

  @SuppressWarnings("removal")
  public IntakeSubsystem() {
    // Configure intake roller motor
    intakeRollerMotor = new SparkMax(Constants.IntakeConstants.kRollerMotorId, MotorType.kBrushless);
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40);
  intakeRollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeRollerEncoder = intakeRollerMotor.getEncoder();

    // Configure intake pivot motor
    intakePivotMotor = new SparkMax(Constants.IntakeConstants.kPivotMotorId, MotorType.kBrushless);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(10);
    
    // Closed loop controller for position
    pivotConfig.closedLoop
        .pidf(25, 0, 0, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(false);

  intakePivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakePivotEncoder = intakePivotMotor.getEncoder();
    intakePivotClosedLoopController = intakePivotMotor.getClosedLoopController();
  }

  /**
   * Command to run the intake while held.
   */
  public Command intakeCommand() {
    return Commands.run(() -> intakeRollerMotor.set(INTAKE_SPEED), this)
        .finallyDo(() -> intakeRollerMotor.set(0))
        .withName("Intake.Run");
  }

  /**
   * Command to eject while held.
   */
  public Command ejectCommand() {
    return Commands.run(() -> intakeRollerMotor.set(-INTAKE_SPEED), this)
        .finallyDo(() -> intakeRollerMotor.set(0))
        .withName("Intake.Eject");
  }

  public Command setPivotAngle(Angle angle) {
    return Commands.runOnce(() -> intakePivotClosedLoopController.setReference(angle, 
        com.revrobotics.spark.SparkBase.ControlType.kPosition), this)
        .withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> intakePivotEncoder.setPosition(0), this)
        .withName("IntakePivot.Rezero");
  }

  /**
   * Command to deploy intake and run roller while held.
   * Stops roller when released.
   */
  public Command deployAndRollCommand() {
    return Commands.run(() -> {
      setIntakeDeployed();
      intakeRollerMotor.set(INTAKE_SPEED);
    }, this).finallyDo(() -> {
      intakeRollerMotor.set(0);
      setIntakeHold();
    }).withName("Intake.DeployAndRoll");
  }

  public Command backFeedAndRollCommand() {
    return Commands.run(() -> {
      setIntakeDeployed();
      intakeRollerMotor.set(-INTAKE_SPEED);
    }, this).finallyDo(() -> {
      intakeRollerMotor.set(0);
      setIntakeHold();
    }).withName("Intake.BackFeedAndRoll");
  }

  @SuppressWarnings("removal")
  private void setIntakeStow() {
    intakePivotClosedLoopController.setReference(0, 
        com.revrobotics.spark.SparkBase.ControlType.kPosition);
  }

  @SuppressWarnings("removal")
  private void setIntakeFeed() {
    intakePivotClosedLoopController.setReference(59, 
        com.revrobotics.spark.SparkBase.ControlType.kPosition);
  }

  @SuppressWarnings("removal")
  private void setIntakeHold() {
    intakePivotClosedLoopController.setReference(115, 
        com.revrobotics.spark.SparkBase.ControlType.kPosition);
  }

  @SuppressWarnings("removal")
  private void setIntakeDeployed() {
    intakePivotClosedLoopController.setReference(148, 
        com.revrobotics.spark.SparkBase.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // Periodic code here if needed
  }

  @Override
  public void simulationPeriodic() {
    // Simulation code here if needed
  }
}
