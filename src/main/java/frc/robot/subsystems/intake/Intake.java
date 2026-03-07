package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public Intake() {
    motor = new SparkMax(intakeMotorCanId, MotorType.kBrushless);
    encoder = motor.getEncoder();
    var config = new SparkMaxConfig();
    config
        .inverted(intakeMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(intakeCurrentLimit)
        .voltageCompensation(12.0);
    config.encoder.velocityConversionFactor(1.0);

    motor.configure(
        config,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  public void intake() {
    motor.setVoltage(intakeVoltage);
  }

  public void outtake() {
    motor.setVoltage(outtakeVoltage);
  }

  public void feed() {
    motor.setVoltage(feedVoltage);
  }

  public void stop() {
    motor.stopMotor();
  }

  public double getVelocityRPM() {
    return encoder.getVelocity();
  }
}
