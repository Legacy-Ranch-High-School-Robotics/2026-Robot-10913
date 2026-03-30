package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.ElasticTelemetry;

public class Shooter extends SubsystemBase {
  private final SparkFlex topMotor;

  private final RelativeEncoder topEncoder;

  private final SparkClosedLoopController topController;

  private double targetVelocityRPM = 90.0;

  private java.util.function.DoubleSupplier distanceSupplier;
  private java.util.function.Supplier<edu.wpi.first.math.geometry.Rotation2d> angleSupplier;

  public Shooter() {
    topMotor = new SparkFlex(topMotorCanId, MotorType.kBrushless);

    topEncoder = topMotor.getEncoder();

    topController = topMotor.getClosedLoopController();

    var topConfig = new SparkFlexConfig();
    topConfig
        .inverted(topMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(shooterCurrentLimit)
        .voltageCompensation(12.0);
    // Velocity Conversion Factor makes encoder.getVelocity() return actual flywheel RPM
    topConfig.encoder.velocityConversionFactor(1.0 / shooterGearRatio);
    topConfig.closedLoop.pid(shooterKp, shooterKi, shooterKd);
    topConfig.closedLoop.feedForward.kV(shooterKv);

    topMotor.configure(
        topConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    // Publishs the default target RPM so it can be edited from Elastic
    ElasticTelemetry.setNumber("Shooter/Target RPM", shooterRPM);
  }

  @Override
  public void periodic() {
    ElasticTelemetry.setNumber("Shooter/Actual RPM", topEncoder.getVelocity());
    ElasticTelemetry.setNumber("Shooter/Target RPM Setpoint", targetVelocityRPM);

    if (distanceSupplier != null) {
      double distanceMeters = distanceSupplier.getAsDouble();
      ElasticTelemetry.setNumber(
          "Shooter/Distance To Hub (m)", Math.round(distanceMeters * 100.0) / 100.0);
      ElasticTelemetry.setNumber(
          "Shooter/Suggested RPM", Math.round(getRPMForDistance(distanceMeters) * 100.0) / 100.0);
    }
    if (angleSupplier != null) {
      double angleErrorDeg = angleSupplier.get().getDegrees();
      ElasticTelemetry.setNumber(
          "Shooter/Angle Error To Hub (deg)", Math.round(angleErrorDeg * 100.0) / 100.0);
    }
  }

  public void setDistanceSupplier(java.util.function.DoubleSupplier distanceSupplier) {
    this.distanceSupplier = distanceSupplier;
  }

  public void setAngleSupplier(
      java.util.function.Supplier<edu.wpi.first.math.geometry.Rotation2d> angleSupplier) {
    this.angleSupplier = angleSupplier;
  }

  public void setVelocity(double velocityRPM) {
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
    return true;
    // return Math.abs(topEncoder.getVelocity() - targetVelocityRPM) < shooterToleranceRPM;
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

  /** Returns the shooter motor (for simulation access). */
  public SparkFlex getTopMotor() {
    return topMotor;
  }

  /**
   * Estimates the required shooter RPM for a given distance to the hub.
   *
   * @param distance Distance to the hub (in meters)
   * @return The interpolated target RPM based on measured values
   */
  public double getRPMForDistance(double distance) {
    return distanceToRpmMap.get(distance);
  }
}
