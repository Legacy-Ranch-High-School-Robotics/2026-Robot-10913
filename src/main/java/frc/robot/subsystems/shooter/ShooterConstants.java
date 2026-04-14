package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * SHOOTER SUBSYSTEM OPERATION:
 *
 * <p>The shooter is a dual-flywheel mechanism that launches game pieces at high velocity.
 *
 * <p>MOTORS: Two SparkFlex NEO Vortex motors (CAN IDs 13 and 14) - Motor Two follows Motor One -
 * Uses closed-loop velocity control (PID + feedforward) - Encoder measures flywheel RPM to maintain
 * consistent shot velocity - Higher RPM = longer shooting distance
 *
 * <p>VELOCITY CONTROL: - PID controller adjusts motor voltage to maintain target RPM - Feedforward
 * (Kv) provides baseline voltage proportional to desired speed - Very low Kp (0.0001) because
 * feedforward does most of the work - atTargetVelocity() checks if actual RPM is within tolerance
 * before feeding
 *
 * <p>DISTANCE-TO-RPM MAPPING: - InterpolatingDoubleTreeMap automatically interpolates between
 * measured data points - Key = distance to target (meters), Value = required shooter RPM - Tune
 * these values by testing shots at known distances and recording successful RPMs
 *
 * <p>SHOOTING MODES: 1. MANUAL: Operator sets fixed RPM via presets (speaker/amp/trap) 2.
 * AUTO-SHOOT: Vision calculates distance and looks up required RPM 3. SHOOT-ON-MOVE: Compensates
 * for robot velocity while moving
 *
 * <p>TYPICAL SEQUENCE: 1. Spin up shooter to target RPM based on distance or preset 2. Wait for
 * atTargetVelocity() to return true (within tolerance) 3. Trigger operator controller rumble to
 * signal ready 4. Feed game piece through hopper into spinning flywheel 5. Game piece is launched
 * toward target
 */
public class ShooterConstants {
  public static final int shooterMotorOneCanId = 13; // DEBUG:SHOOTER_MOTOR_ONE_CAN_ID
  public static final int shooterMotorTwoCanId = 14; // DEBUG:SHOOTER_MOTOR_TWO_CAN_ID

  public static final boolean shooterMotorOneInverted = false; // DEBUG:SHOOTER_MOTOR_ONE_INVERTED
  public static final boolean shooterMotorTwoInverted = false; // DEBUG:SHOOTER_MOTOR_TWO_INVERTED

  public static final int shooterCurrentLimit = 60; // DEBUG:SHOOTER_CURRENT_LIMIT

  public static final double shooterKp = 0.0001; // DEBUG:SHOOTER_KP
  public static final double shooterKi = 0.0; // DEBUG:SHOOTER_KI
  public static final double shooterKd = 0.0; // DEBUG:SHOOTER_KD
  public static final double shooterKv = 0.0021; // DEBUG:SHOOTER_KV

  public static final double shooterRPM = 3800.0; // DEBUG:SHOOTER_DEFAULT_RPM
  public static final double shooterRPMInverted = -2000.0; // DEBUG:SHOOTER_EJECT_RPM

  // THESE PRESETS ARE NOT FULLY TESTED //NOTE SOMETHING IS CAUSING A REDUCTION IN RPM SO I
  // INCREASED RPM
  public static final double closePresetRPM = 3150.0; // DEBUG:SHOOTER_CLOSE_PRESET
  public static final double atMediumPresetRPM = 3800.0; // DEBUG: SHOOTER_MEDIUM_PRESET
  public static final double atDistancePresetRPM = 4750.0; // DEBUG:SHOOTER_DISTANCE_PRESET

  // Tolerance: Maximum RPM error to consider the shooter "at target velocity".
  // If abs(actualRPM - targetRPM) < tolerance, the shooter is ready to launch.
  // Tighter tolerance = more consistent shots but longer spin-up wait time.
  // 150 RPM tolerance provides good balance between accuracy and responsiveness.
  public static final double shooterToleranceRPM = 75.0; // DEBUG:SHOOTER_TOLERANCE

  // Distance-to-RPM lookup table: Automatically interpolates between data points.
  // Add more entries by testing shots at known distances and recording successful RPMs.
  // Format: distanceToRpmMap.put(distanceInMeters, requiredRPM);
  public static final InterpolatingDoubleTreeMap distanceToRpmMap =
      new InterpolatingDoubleTreeMap();

  static {
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(2.5), 3250.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(3.0), 3400.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(3.5), 3500.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(4.0), 3625.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(4.5), 3700.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(5.0), 3850.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(5.5), 3900.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(6.0), 4000.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(6.5), 4050.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(7.0), 4150.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(7.5), 4225.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(8.0), 4300.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(8.5), 4375.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(9.0), 4450.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(9.5), 4525.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(10.0), 4600.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(10.5), 4675.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(11.0), 4750.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(11.5), 4825.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(12.0), 4900.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(12.5), 4975.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(13.0), 5050.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(13.5), 5100.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(14.0), 5175.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(14.5), 5225.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(15.0), 5300.0);
  }
}
