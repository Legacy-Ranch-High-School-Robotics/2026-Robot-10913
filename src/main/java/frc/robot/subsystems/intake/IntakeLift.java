package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeLiftConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeLift extends SubsystemBase {
  private final SparkMax liftMotor;
  private final RelativeEncoder liftEncoder;

  public IntakeLift() {
    liftMotor = new SparkMax(intakeLiftMotorCanId, MotorType.kBrushless);
    liftEncoder = liftMotor.getEncoder();

    var liftConfig = new SparkMaxConfig();
    liftConfig
        .inverted(intakeLiftMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(intakeCurrentLimit)
        .voltageCompensation(12.0);

    liftMotor.configure(
        liftConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  public void deploy() {
    if (getPosition() < deployedPosition + liftPositionTolerance) {
      liftMotor.setVoltage(liftVoltage);
    } else {
      stop();
    }
  }

  public void retract() {
    if (getPosition() > retractedPosition - liftPositionTolerance) {
      liftMotor.setVoltage(liftVoltage * -1);
    } else {
      stop();
    }
  }

  public void stop() {
    liftMotor.stopMotor();
  }

  public double getPosition() {
    return liftEncoder.getPosition();
  }

  public boolean isDeployed() {
    return Math.abs(getPosition() - deployedPosition) < liftPositionTolerance;
  }

  public boolean isRetracted() {
    return Math.abs(getPosition() - retractedPosition) < liftPositionTolerance;
  }
}
