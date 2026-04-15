package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final SparkMax motor;

  private final SparkMax liftMotor;

  private final RelativeEncoder encoder;

  // lift motor and encoder

  private final RelativeEncoder liftEncoder;
  
  // Notifier for time-based deployment
  private Notifier m_deployNotifier;
  private boolean m_isDeploying = false;
  private boolean m_isRetracting = false;

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
  public void periodic() {
    // Publish intake deployment state telemetry
    ElasticTelemetry.setBoolean("Intake/IsDeployed", isLiftDeployed());
    ElasticTelemetry.setBoolean("Intake/IsRetracted", isLiftRetracted());
    ElasticTelemetry.setBoolean("Intake/IsDeploying", isDeploying());
    ElasticTelemetry.setBoolean("Intake/IsRetracting", isRetracting());
  }

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
  
  /** Retract intake for specific time using constants */
  public void retractTimed() {
    if (m_isDeploying || m_isRetracting) return;
    
    m_isRetracting = true;
    liftMotor.setVoltage(-liftVoltage);
    
    m_deployNotifier = new Notifier(() -> {
      liftStop();
      m_isRetracting = false;
    });
    m_deployNotifier.startSingle(retractTimeSeconds);
  }

  public void liftDeploy() {

    if (getLiftPosition() < deployedPosition + liftPositionTolerance) {

      liftMotor.setVoltage(liftVoltage);

    } else {

      liftStop();
    }
  }
  
  /** Deploy intake for specific time using constants */
  public void deployTimed() {
    if (m_isDeploying || m_isRetracting) return;
    
    m_isDeploying = true;
    liftMotor.setVoltage(liftVoltage);
    
    m_deployNotifier = new Notifier(() -> {
      liftStop();
      m_isDeploying = false;
    });
    m_deployNotifier.startSingle(deployTimeSeconds);
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
    if (m_deployNotifier != null) {
      m_deployNotifier.stop();
      m_deployNotifier = null;
    }
  }
  
  public boolean isDeploying() {
    return m_isDeploying;
  }
  
  public boolean isRetracting() {
    return m_isRetracting;
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
}
