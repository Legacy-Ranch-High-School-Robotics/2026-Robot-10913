package frc.robot.subsystems.intake;

/**
 * INTAKE SUBSYSTEM OPERATION:
 *
 * <p>The intake subsystem consists of two mechanisms: 1. ROLLER (SparkMax NEO550): Spins to pull
 * game pieces into the robot or eject them. - Runs at fixed voltages for intake/outtake/feed
 * operations - No closed-loop control, just voltage commands
 *
 * <p>2. LIFT (SparkMax NEO550): Raises/lowers the intake using a motor with encoder feedback. -
 * Uses position-based control to deploy (extend) or retract the intake - Encoder tracks rotations
 * to determine lift position - Position tolerance determines when the lift has reached its target
 *
 * <p>TYPICAL OPERATION SEQUENCE: 1. Deploy lift to extended position (deployedPosition) 2. Run
 * roller forward to intake game pieces 3. Retract lift back to stowed position (retractedPosition)
 * 4. Feed game piece to shooter/hopper when ready
 */
public class IntakeConstants {

  public static final int intakeMotorCanId = 11; // DEBUG:INTAKE_ROLLER_CAN_ID
  public static final int intakeLiftMotorCanId = 10; // DEBUG:INTAKE_LIFT_CAN_ID

  public static final boolean intakeMotorInverted = false; // DEBUG:INTAKE_ROLLER_INVERTED

  public static final boolean intakeLiftMotorInverted = false; // DEBUG:INTAKE_LIFT_INVERTED

  public static final int intakeCurrentLimit = 40; // DEBUG:INTAKE_CURRENT_LIMIT

  public static final double intakeVoltage = 10.0; // DEBUG:INTAKE_VOLTAGE

  public static final double outtakeVoltage = -8.0; // DEBUG:OUTTAKE_VOLTAGE

  public static final double liftVoltage = 1.25; // DEBUG:LIFT_VOLTAGE

  public static final double feedVoltage = 6.0; // DEBUG:INTAKE_FEED_VOLTAGE

  public static final double deployedPosition = 3.476; // DEBUG:LIFT_DEPLOYED_POSITION

  public static final double retractedPosition = 0; // DEBUG:LIFT_RETRACTED_POSITION

  // Tolerance: Maximum position error (in encoder rotations) to consider the lift "at target".
  // If abs(currentPosition - targetPosition) < tolerance, the lift is considered in position.
  public static final double liftPositionTolerance = 0.5; // DEBUG:LIFT_TOLERANCE

  // Soft Limits (NEED TO TEST!!!!)
  public static final double liftUpperLimit = 4.0; // DEBUG:LIFT_UPPER_LIMIT
  public static final double liftLowerLimit = -0.5; // DEBUG:LIFT_LOWER_LIMIT
}
