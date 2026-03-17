package frc.robot.subsystems.intakelift;

import static frc.robot.subsystems.intakelift.IntakeLiftConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeLift extends SubsystemBase {

  private final SparkMax liftMotor;

  // lift motor and encoder

  private final RelativeEncoder liftEncoder;

  private static boolean isDeployed = false;

  private static boolean isRetracted = false;

  public IntakeLift() {

    liftMotor = new SparkMax(intakeLiftMotorCanId, MotorType.kBrushless);

    liftEncoder = liftMotor.getEncoder();

    var liftConfig = new SparkMaxConfig();

    liftConfig
        .inverted(intakeLiftMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(intakeLiftCurrentLimit)
        .voltageCompensation(12.0);

    liftConfig.encoder.velocityConversionFactor(1.0);

    liftMotor.configure(
        liftConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  public void liftRetract() {

    if (getLiftPosition() > retractedPosition + liftPositionTolerance) {
      liftMotor.setVoltage(liftVoltage * -1);
    } else {
      isDeployed = false;
      isRetracted = true;
      liftStop();
    }
  }

  public void liftDeploy() {

    if (getLiftPosition() < deployedPosition - liftPositionTolerance) {
      liftMotor.setVoltage(liftVoltage);
    } else {
      isDeployed = true;
      isRetracted = false;
      liftStop();
    }
  }

  public double getLiftPosition() {
    return liftEncoder.getPosition();
  }

  public boolean isLiftDeployed() {
    return isDeployed;
  }

  public boolean isLiftRetracted() {
    return isRetracted;
  }

  public void liftStop() {
    liftMotor.stopMotor();
  }
}
