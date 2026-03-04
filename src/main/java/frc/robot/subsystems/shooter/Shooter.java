package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;

public class Shooter extends SubsystemBase {
  private final SparkFlex topMotor;
  private final SparkMax bottomMotor;

  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  private final SparkClosedLoopController topController;
  private final SparkClosedLoopController bottomController;

  private double targetVelocityRPM = 0.0;

  public Shooter() {
    topMotor = new SparkFlex(topMotorCanId, MotorType.kBrushless);
    bottomMotor = new SparkMax(bottomMotorCanId, MotorType.kBrushless);

    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();

    topController = topMotor.getClosedLoopController();
    bottomController = bottomMotor.getClosedLoopController();

    var topConfig = new SparkMaxConfig();
    topConfig
        .inverted(topMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(shooterCurrentLimit)
        .voltageCompensation(12.0);
    topConfig.encoder.velocityConversionFactor(1.0);
    topConfig.closedLoop.pid(shooterKp, shooterKi, shooterKd);
    topConfig.closedLoop.velocityFF(shooterKv);

    var bottomConfig = new SparkMaxConfig();
    bottomConfig
        .inverted(bottomMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(shooterCurrentLimit)
        .voltageCompensation(12.0);
    bottomConfig.encoder.velocityConversionFactor(1.0);
    bottomConfig.closedLoop.pid(shooterKp, shooterKi, shooterKd);
    bottomConfig.closedLoop.velocityFF(shooterKv);

    topMotor.configure(
        topConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    bottomMotor.configure(
        bottomConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  public void setVelocity(double velocityRPM) {
    targetVelocityRPM = velocityRPM;
    topController.setSetpoint(velocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    bottomController.setSetpoint(velocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void setVoltage(double volts) {
    topMotor.setVoltage(volts);
    bottomMotor.setVoltage(volts);
  }

  public void stop() {
    targetVelocityRPM = 0.0;
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  public boolean atTargetVelocity() {
    return Math.abs(topEncoder.getVelocity() - targetVelocityRPM) < shooterToleranceRPM
        && Math.abs(bottomEncoder.getVelocity() - targetVelocityRPM) < shooterToleranceRPM;
  }

  public double getAverageVelocityRPM() {
    return (topEncoder.getVelocity() + bottomEncoder.getVelocity()) / 2.0;
  }

  public double getTargetVelocityRPM() {
    return targetVelocityRPM;
  }

  public double getTopVelocityRPM() {
    return topEncoder.getVelocity();
  }

  public double getBottomVelocityRPM() {
    return bottomEncoder.getVelocity();
  }
}
