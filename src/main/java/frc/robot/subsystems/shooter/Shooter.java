package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.ElasticTelemetry;

public class Shooter extends SubsystemBase {
  private final SparkFlex topMotor;

  private final RelativeEncoder topEncoder;

  private final SparkClosedLoopController topController;

  private double targetVelocityRPM = 90.0;

  public Shooter() {
    topMotor = new SparkFlex(topMotorCanId, MotorType.kBrushless);

    topEncoder = topMotor.getEncoder();

    topController = topMotor.getClosedLoopController();

    var topConfig = new SparkMaxConfig();
    topConfig
        .inverted(topMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(shooterCurrentLimit)
        .voltageCompensation(12.0);
    topConfig.encoder.velocityConversionFactor(1.0);
    topConfig.closedLoop.pid(shooterKp, shooterKi, shooterKd);
    topConfig.closedLoop.velocityFF(shooterKv);

    topMotor.configure(
        topConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    // Publish the default target RPM so it can be edited from Elastic
    ElasticTelemetry.setNumber("Shooter/Target RPM", shooterRPM);
  }

  @Override
  public void periodic() {
    ElasticTelemetry.setNumber("Shooter/Actual RPM", topEncoder.getVelocity());
    ElasticTelemetry.setNumber("Shooter/Target RPM Setpoint", targetVelocityRPM);
  }

  public void setVelocity(double velocityRPM) {
    targetVelocityRPM = velocityRPM;
    topController.setSetpoint(velocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void setTopVelocity(double velocityRPM) {
    targetVelocityRPM = velocityRPM;
    topController.setSetpoint(velocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void setVoltage(double volts) {
    topMotor.setVoltage(volts);
  }

  public void stop() {
    targetVelocityRPM = 0.0;
    topMotor.stopMotor();
  }

  public void eject() {
    topMotor.setVoltage(-6.0);
  }

  public boolean atTargetVelocity() {
    System.out.println(topEncoder.getVelocity());
    return Math.abs(topEncoder.getVelocity() - targetVelocityRPM) < shooterToleranceRPM;
  }

  public boolean atTestingVelocity() {
    return false;
  }

  public double getVelocityRPM() {
    return topEncoder.getVelocity();
  }

  public double getTargetVelocityRPM() {
    return targetVelocityRPM;
  }
}
