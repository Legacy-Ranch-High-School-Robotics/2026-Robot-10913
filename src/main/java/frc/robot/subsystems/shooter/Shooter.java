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
  private final SparkFlex shooterMotorOne;
  private final SparkFlex shooterMotorTwo;

  private final RelativeEncoder shooterMotorOneEncoder;

  private final SparkClosedLoopController shooterMotorOneController;

  private java.util.function.DoubleSupplier distanceSupplier;
  private java.util.function.Supplier<edu.wpi.first.math.geometry.Rotation2d> angleSupplier;

  private double targetRPM = 0.0;

  public Shooter() {
    shooterMotorOne = new SparkFlex(shooterMotorOneCanId, MotorType.kBrushless);
    shooterMotorTwo = new SparkFlex(shooterMotorTwoCanId, MotorType.kBrushless);

    shooterMotorOneEncoder = shooterMotorOne.getEncoder();

    shooterMotorOneController = shooterMotorOne.getClosedLoopController();

    var motorOneConfig = new SparkFlexConfig();
    motorOneConfig
        .inverted(shooterMotorOneInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(shooterCurrentLimit)
        .voltageCompensation(12.0);
    // Gear ratio: 3:2 (motor shaft @ 4000 RPM → flywheel @ 2000 RPM)
    // Conversion factor = flywheel RPM / motor RPM = 2000 / 4000 = 0.5
    motorOneConfig.encoder.velocityConversionFactor(0.667);
    motorOneConfig.closedLoop.pid(shooterKp, shooterKi, shooterKd);
    motorOneConfig.closedLoop.feedForward.kV(shooterKv);

    shooterMotorOne.configure(
        motorOneConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    var motorTwoConfig = new SparkFlexConfig();
    motorTwoConfig
        .inverted(shooterMotorTwoInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(shooterCurrentLimit)
        .voltageCompensation(12.0)
        .follow(shooterMotorOneCanId, true);
    motorTwoConfig.encoder.velocityConversionFactor(0.667);
    motorTwoConfig.closedLoop.pid(shooterKp, shooterKi, shooterKd);
    motorTwoConfig.closedLoop.feedForward.kV(shooterKv);

    shooterMotorTwo.configure(
        motorTwoConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    // Publishs the default target RPM so it can be edited from Elastic
    ElasticTelemetry.setNumber("Shooter/Target RPM", shooterRPM);
  }

  @Override
  public void periodic() {
    ElasticTelemetry.setNumber("Shooter/Actual RPM", shooterMotorOneEncoder.getVelocity());

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
    targetRPM = velocityRPM; // store it
    shooterMotorOneController.setSetpoint(
        velocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void setVoltage(double volts) {
    shooterMotorOne.setVoltage(volts);
  }

  public void stop() {
    shooterMotorOne.stopMotor();
  }

  public void eject() {
    shooterMotorOne.setVoltage(-6.0);
  }

  public boolean atTargetVelocity() {
    return targetRPM > 0 && Math.abs(getVelocityRPM() - targetRPM) < shooterToleranceRPM;
  }

  public double getVelocityRPM() {
    return shooterMotorOneEncoder.getVelocity();
  }

  /**
   * Estimates the required shooter RPM for a given distance to the hub.
   *
   * @param distance Distance to the hub (in meters)
   * @return The interpolated target RPM based on measured values
   */
  double lastKnownRPM = 0.0;

  public double getRPMForDistance(double distance) {
    return targetRPM = distanceToRpmMap.get(distance);
  }
}
