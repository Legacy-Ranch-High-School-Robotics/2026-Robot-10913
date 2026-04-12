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

  private final SparkMax liftMotor;

  private final RelativeEncoder encoder;

  // lift motor and encoder

  private final RelativeEncoder liftEncoder;

  public Intake() {

    motor = new SparkMax(intakeMotorCanId, MotorType.kBrushless);

    liftMotor = new SparkMax(intakeLiftMotorCanId, MotorType.kBrushless);

    encoder = motor.getEncoder();

    liftEncoder = liftMotor.getEncoder();

    var config = new SparkMaxConfig();

    config
        .inverted(intakeMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(intakeCurrentLimit)
        .voltageCompensation(12.0);

    config.encoder.velocityConversionFactor(1.0);

    var liftConfig = new SparkMaxConfig();

    liftConfig
        .inverted(intakeLiftMotorInverted)
        // idleMode(IdleMode.kBrake)
        // dleMode(IdleMode.kCoast)
        .smartCurrentLimit(intakeCurrentLimit)
        .voltageCompensation(12.0);

    motor.configure(
        config,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    liftMotor.configure(
        liftConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  public void intake() {

    motor.setVoltage(intakeVoltage);
  }

  public void liftRetract() {

    if (getLiftPosition() > retractedPosition - liftPositionTolerance) {

      liftMotor.setVoltage(-liftVoltage);

    } else {

      liftStop();
    }
  }

  public void liftDeploy() {

    if (getLiftPosition() < deployedPosition + liftPositionTolerance) {

      liftMotor.setVoltage(liftVoltage);

    } else {

      liftStop();
    }
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

  public void liftStop() {

    liftMotor.stopMotor();
  }

  public double getVelocityRPM() {

    return encoder.getVelocity();
  }

  public double getLiftPosition() {

    return liftEncoder.getPosition();
  }

  public boolean isLiftDeployed() {

    return Math.abs(getLiftPosition() - deployedPosition) < liftPositionTolerance;
  }

  public boolean isLiftRetracted() {

    return Math.abs(getLiftPosition() - retractedPosition) < liftPositionTolerance;
  }

  public SparkMax getMotor() {
    return motor;
  }

  public SparkMax getLiftMotor() {
    return liftMotor;
  }
}
