package frc.robot.subsystems.hopper;

/**
 * HOPPER SUBSYSTEM OPERATION:
 *
 * <p>The hopper is a single-motor mechanism that feeds game pieces from the intake to the shooter.
 *
 * <p>MOTOR: SparkMax NEO (CAN ID 12) - Uses closed-loop velocity control (PID + feedforward) -
 * Encoder measures wheel RPM to maintain consistent feeding speed - Runs at different speeds
 * depending on operation mode
 *
 * <p>OPERATION MODES: 1. IDLE: Slow rotation (1500 RPM) to stage game pieces without shooting 2.
 * FEED: Fast rotation (5000 RPM) to rapidly feed pieces into the shooter 3. EJECT: Reverse voltage
 * (-6V) to clear jammed pieces
 *
 * <p>VELOCITY CONTROL: - PID controller maintains target RPM by adjusting motor voltage -
 * Feedforward (Kv) provides baseline voltage based on desired speed - atTargetVelocity() checks if
 * actual RPM is within tolerance of setpoint
 *
 * <p>TYPICAL SEQUENCE: 1. Intake delivers game piece to hopper 2. Hopper idles at low speed to hold
 * piece in position 3. When shooter reaches target velocity, hopper feeds at high speed 4. Game
 * piece is launched from shooter
 */
public class HopperConstants {
  public static final int hopperMotorCanId = 12; // DEBUG:HOPPER_CAN_ID
  public static final boolean hopperMotorInverted = false; // DEBUG:HOPPER_INVERTED
  public static final int hopperCurrentLimit = 60; // DEBUG:HOPPER_CURRENT_LIMIT

  public static final double hopperKp = 0.0001;
  public static final double hopperKi = 0.0;
  public static final double hopperKd = 0.0;
  public static final double hopperKv = 0.0021;

  public static final double hopperIdleRPM = 1500.0; // DEBUG:HOPPER_IDLE_RPM
  public static final double hopperFeedRPM = 5000.0; // DEBUG:HOPPER_FEED_RPM

  // Tolerance: Maximum RPM error to consider the hopper "at target velocity".
  // If abs(actualRPM - targetRPM) < tolerance, the hopper is ready to feed.
  // Used to ensure consistent feeding speed before launching game pieces.
  public static final double hopperToleranceRPM = 200.0; // DEBUG:HOPPER_TOLERANCE
}
