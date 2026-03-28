package frc.robot.subsystems.intake;

/**
 * INTAKE SUBSYSTEM OPERATION:
 * 
 * The intake subsystem consists of two mechanisms:
 * 1. ROLLER (SparkMax NEO550): Spins to pull game pieces into the robot or eject them.
 *    - Runs at fixed voltages for intake/outtake/feed operations
 *    - No closed-loop control, just voltage commands
 * 
 * 2. LIFT (SparkMax NEO550): Raises/lowers the intake using a motor with encoder feedback.
 *    - Uses position-based control to deploy (extend) or retract the intake
 *    - Encoder tracks rotations to determine lift position
 *    - Position tolerance determines when the lift has reached its target
 * 
 * TYPICAL OPERATION SEQUENCE:
 * 1. Deploy lift to extended position (deployedPosition)
 * 2. Run roller forward to intake game pieces
 * 3. Retract lift back to stowed position (retractedPosition)
 * 4. Feed game piece to shooter/hopper when ready
 */
public class IntakeConstants {

  public static final int intakeMotorCanId = 11;
  public static final int intakeLiftMotorCanId = 10;

  public static final boolean intakeMotorInverted = false;

  public static final boolean intakeLiftMotorInverted = false;

  public static final int intakeCurrentLimit = 40;

  public static final double intakeVoltage = 10.0;

  public static final double outtakeVoltage = -8.0;

  public static final double liftVoltage = 1.25;

  public static final double feedVoltage = 6.0;

  public static final double deployedPosition = 3.476;

  public static final double retractedPosition = 0;

  // Tolerance: Maximum position error (in encoder rotations) to consider the lift "at target".
  // If abs(currentPosition - targetPosition) < tolerance, the lift is considered in position.
  public static final double liftPositionTolerance = 0.5;

  //Soft Limits (NEED TO TEST!!!!)
  public static final double liftUpperLimit = 4.0;  
  public static final double liftLowerLimit = -0.5; 
}
