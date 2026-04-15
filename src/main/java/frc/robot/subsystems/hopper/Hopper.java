package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.ElasticTelemetry;

public class Hopper extends SubsystemBase {
  private final SparkMax hopperMotor;
  private final RelativeEncoder hopperEncoder;
  private final SparkClosedLoopController hopperController;

  private double targetVelocityRPM = 0.0;

  public Hopper() {
    hopperMotor = new SparkMax(hopperMotorCanId, MotorType.kBrushless);
    hopperEncoder = hopperMotor.getEncoder();
    hopperController = hopperMotor.getClosedLoopController();

    var hopperConfig = new SparkMaxConfig();
    hopperConfig
        .inverted(hopperMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(hopperCurrentLimit)
        .voltageCompensation(12.0);
    hopperConfig.encoder.velocityConversionFactor(1.0);
    hopperConfig.closedLoop.pid(hopperKp, hopperKi, hopperKd);
    hopperConfig.closedLoop.feedForward.kV(hopperKv);

    hopperMotor.configure(
        hopperConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    ElasticTelemetry.setNumber("Hopper/Actual RPM", hopperEncoder.getVelocity());
    ElasticTelemetry.setNumber("Hopper/Target RPM", targetVelocityRPM);
    ElasticTelemetry.setBoolean("Hopper/AtTargetVelocity", atTargetVelocity());
  }

  public void setVelocity(double velocityRPM) {
    targetVelocityRPM = velocityRPM;
    hopperController.setSetpoint(velocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void setVoltage(double volts) {
    hopperMotor.setVoltage(volts);
  }

  public void stop() {
    targetVelocityRPM = 0.0;
    hopperMotor.stopMotor();
  }

  public void eject() {
    hopperMotor.setVoltage(-6.0);
  }

  public boolean atTargetVelocity() {
    return Math.abs(hopperEncoder.getVelocity() - targetVelocityRPM) < hopperToleranceRPM;
  }

  public double getVelocityRPM() {
    return hopperEncoder.getVelocity();
  }

  public double getTargetVelocityRPM() {
    return targetVelocityRPM;
  }
}
