package frc.robot.subsystems.fuel;

public class FuelConstants {
  // Motor controller IDs for Fuel Mechanism motors
  // TODO: Update these IDs to match your robot
  public static final int leftIntakeLauncherMotorId = 5;
  public static final int rightIntakeLauncherMotorId = 6;
  public static final int indexerMotorId = 8;

  // Current limit for fuel mechanism motors.
  public static final int indexerMotorCurrentLimit = 80;
  public static final int launcherMotorCurrentLimit = 80;

  // All values likely need to be tuned based on your robot
  public static final double indexerIntakingPercent = -.8;
  public static final double indexerLaunchingPercent = 0.6;
  public static final double indexerSpinUpPreLaunchPercent = -0.5;

  public static final double intakeIntakingPercent = 0.6;
  public static final double launchingLauncherPercent = .85;
  public static final double intakeEjectPercent = -0.8;

  public static final double spinUpSeconds = 0.75;

  // Rotation factors for encoders
  public static final double encoderPositionFactor = 2 * Math.PI;
  public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0;
}
