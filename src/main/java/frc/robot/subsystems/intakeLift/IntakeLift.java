package frc.robot.subsystems.intakelift;

import static frc.robot.subsystems.intakelift.IntakeLiftConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeLift extends SubsystemBase {

  public enum HomingState {
    FINDING_ZERO,
    FINDING_MAX,
    HOMED
  }

  private HomingState homingState = HomingState.FINDING_ZERO;
  private double dynamicDeployedPosition = deployedPosition;

  private final SparkMax liftMotor;

  // lift motor and encoder

  private final RelativeEncoder liftEncoder;

  private static boolean isDeployed = false;

  private static boolean isRetracted = false;

  public IntakeLift() {
    if (Preferences.containsKey("IntakeLift_DeployedPosition")) {
      dynamicDeployedPosition =
          Preferences.getDouble("IntakeLift_DeployedPosition", deployedPosition);
      homingState = HomingState.HOMED; // Assume we don't need to rehome if soft limits exist
    }

    liftMotor = new SparkMax(intakeLiftMotorCanId, MotorType.kBrushless);

    liftEncoder = liftMotor.getEncoder();

    var liftConfig = new SparkMaxConfig();

    liftConfig
        .inverted(intakeLiftMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(intakeLiftCurrentLimit)
        .voltageCompensation(12.0);

    liftConfig.encoder.velocityConversionFactor(1.0);

    liftConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit((float) 0) // Max retraction is 0 after homing
        .reverseSoftLimit((float) dynamicDeployedPosition);

    liftMotor.configure(
        liftConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeLift/Position", getLiftPosition());
    SmartDashboard.putNumber("IntakeLift/CurrentAmps", getCurrentAmps());
    SmartDashboard.putString("IntakeLift/HomingState", homingState.toString());
    SmartDashboard.putBoolean("IntakeLift/IsDeployed", isLiftDeployed());
  }

  public void applyVoltage(double voltage) {
    liftMotor.setVoltage(voltage);
  }

  public double getCurrentAmps() {
    return liftMotor.getOutputCurrent();
  }

  public HomingState getHomingState() {
    return homingState;
  }

  public void setHomingState(HomingState state) {
    this.homingState = state;
  }

  public void zeroEncoder() {
    liftEncoder.setPosition(0);
  }

  public void saveMaxPosition() {
    dynamicDeployedPosition = getLiftPosition();
    Preferences.setDouble("IntakeLift_DeployedPosition", dynamicDeployedPosition);

    // Update soft limits
    var liftConfig = new SparkMaxConfig();
    liftConfig.softLimit.reverseSoftLimit((float) dynamicDeployedPosition);
    liftMotor.configure(
        liftConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  public void liftRetract() {
    if (homingState != HomingState.HOMED) return;

    if (getLiftPosition() > 0 + liftPositionTolerance) {
      liftMotor.setVoltage(liftVoltage * -1);
    } else {
      isDeployed = false;
      isRetracted = true;
      liftStop();
    }
  }

  public void liftDeploy() {
    if (homingState != HomingState.HOMED) return;

    if (getLiftPosition() < dynamicDeployedPosition - liftPositionTolerance) {
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
