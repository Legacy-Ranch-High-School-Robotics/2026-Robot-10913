package frc.robot.subsystems.fuel;

import static frc.robot.subsystems.fuel.FuelConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FuelIOHardware implements FuelIO {

  // ==========================================
  // 1. Declare Hardware Objects
  // ==========================================
  private final SparkMax leftIntakeLauncher;
  // TODO: Declare the SparkMax for the rightIntakeLauncher
  // TODO: Declare the SparkMax for the indexer

  private final RelativeEncoder leftEncoder;
  // TODO: Declare the RelativeEncoder for the rightEncoder

  public FuelIOHardware() {
    // ==========================================
    // 2. Initialize Motors
    // ==========================================
    leftIntakeLauncher = new SparkMax(leftIntakeLauncherMotorId, MotorType.kBrushless);
    // TODO: Initialize the rightIntakeLauncher (MotorType.kBrushless)
    // TODO: Initialize the indexer (MotorType.kBrushed)

    // ==========================================
    // 3. Create Configurations
    // ==========================================

    // --- Launcher Configuration ---
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.smartCurrentLimit(launcherMotorCurrentLimit);
    launcherConfig.voltageCompensation(12);
    launcherConfig.idleMode(IdleMode.kCoast);

    launcherConfig.encoder.positionConversionFactor(encoderPositionFactor);
    launcherConfig.encoder.velocityConversionFactor(encoderVelocityFactor);

    // --- Indexer Configuration ---
    // TODO: Create a SparkMaxConfig for the indexer
    // TODO: Set the smartCurrentLimit using indexerMotorCurrentLimit

    // ==========================================
    // 4. Apply Configurations
    // ==========================================

    // The left intake launcher is inverted
    launcherConfig.inverted(true);
    leftIntakeLauncher.configure(
        launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // TODO: The right intake launcher should NOT be inverted.
    // Hint: Change launcherConfig.inverted(false) before applying it!
    // TODO: Apply the right launcher config

    // TODO: Apply the indexer config to the indexer motor

    // ==========================================
    // 5. Initialize Encoders
    // ==========================================
    leftEncoder = leftIntakeLauncher.getEncoder();
    // TODO: Get the encoder for the rightIntakeLauncher
  }

  @Override
  public void updateInputs(FuelIOInputs inputs) {
    // --- Left Launcher ---
    inputs.leftLauncherPositionRad = leftEncoder.getPosition();
    inputs.leftLauncherVelocityRadPerSec = leftEncoder.getVelocity();
    inputs.leftLauncherAppliedVolts =
        leftIntakeLauncher.getAppliedOutput() * leftIntakeLauncher.getBusVoltage();
    inputs.leftLauncherCurrentAmps = leftIntakeLauncher.getOutputCurrent();

    // --- Right Launcher ---
    // TODO: Map the right motor and encoder data to the inputs object

    // --- Indexer ---
    // TODO: Map the indexer motor data to the inputs object
    // Note: Map AppliedVolts and CurrentAmps only.
    // Leave indexerPositionRad and indexerVelocityRadPerSec at 0.0!
  }

  @Override
  // Set the voltage of the intake/launcher roller
  public void setLeftIntakeLauncherRoller(double power) {
    // TODO: Implement this method
  }

  @Override
  // Set the voltage of the intake/launcher roller
  public void setRightIntakeLauncherRoller(double power) {
    // TODO: Implement this method
  }

  // Set the voltage of the feeder roller
  public void setFeederRoller(double power) {
    // TODO: Implement this method
  }

  // Stop the rollers
  public void stop() {
    // TODO: Implement this method
  }
}
