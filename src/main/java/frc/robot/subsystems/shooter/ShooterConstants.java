package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * SHOOTER SUBSYSTEM OPERATION:
 * 
 * The shooter is a single-flywheel mechanism that launches game pieces at high velocity.
 * 
 * MOTOR: SparkFlex NEO Vortex (CAN ID 13)
 * - Uses closed-loop velocity control (PID + feedforward)
 * - Encoder measures flywheel RPM to maintain consistent shot velocity
 * - Higher RPM = longer shooting distance
 * 
 * VELOCITY CONTROL:
 * - PID controller adjusts motor voltage to maintain target RPM
 * - Feedforward (Kv) provides baseline voltage proportional to desired speed
 * - Very low Kp (0.0001) because feedforward does most of the work
 * - atTargetVelocity() checks if actual RPM is within tolerance before feeding
 * 
 * DISTANCE-TO-RPM MAPPING:
 * - InterpolatingDoubleTreeMap automatically interpolates between measured data points
 * - Key = distance to target (meters), Value = required shooter RPM
 * - Tune these values by testing shots at known distances and recording successful RPMs
 * 
 * SHOOTING MODES:
 * 1. MANUAL: Operator sets fixed RPM via presets (speaker/amp/trap)
 * 2. AUTO-SHOOT: Vision calculates distance and looks up required RPM
 * 3. SHOOT-ON-MOVE: Compensates for robot velocity while moving
 * 
 * TYPICAL SEQUENCE:
 * 1. Spin up shooter to target RPM based on distance or preset
 * 2. Wait for atTargetVelocity() to return true (within tolerance)
 * 3. Trigger operator controller rumble to signal ready
 * 4. Feed game piece through hopper into spinning flywheel
 * 5. Game piece is launched toward target
 */
public class ShooterConstants {
  public static final int topMotorCanId = 13;

  public static final boolean topMotorInverted = false;

  public static final int shooterCurrentLimit = 60;

  public static final double shooterKp = 0.0001;
  public static final double shooterKi = 0.0;
  public static final double shooterKd = 0.0;
  public static final double shooterKv = 0.0021;

  public static final double shooterRPM = 4575.0;
  public static final double shooterRPMInverted = -2000.0;

  public static final double closePresetRPM = 3000.0;
  public static final double atDistancePresetRPM = 5200.0;

  // Tolerance: Maximum RPM error to consider the shooter "at target velocity".
  // If abs(actualRPM - targetRPM) < tolerance, the shooter is ready to launch.
  // Tighter tolerance = more consistent shots but longer spin-up wait time.
  // 150 RPM tolerance provides good balance between accuracy and responsiveness.
  public static final double shooterToleranceRPM = 150.0;

  // Distance-to-RPM lookup table: Automatically interpolates between data points.
  // Add more entries by testing shots at known distances and recording successful RPMs.
  // Format: distanceToRpmMap.put(distanceInMeters, requiredRPM);
  public static final InterpolatingDoubleTreeMap distanceToRpmMap =
      new InterpolatingDoubleTreeMap();

  static {
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(2.0), 2000.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(3.0), 2500.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(5.0), 3500.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(7.0), 5000.0);
  }
}
