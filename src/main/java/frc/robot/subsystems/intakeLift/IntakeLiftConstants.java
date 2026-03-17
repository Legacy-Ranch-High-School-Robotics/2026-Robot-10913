package frc.robot.subsystems.intakelift;

public class IntakeLiftConstants {

  // Black Roller Is 11

  public static final int intakeLiftMotorCanId = 10;

  public static final boolean intakeLiftMotorInverted = false;

  public static double liftVoltage = 0.75;

  // Homing constants
  public static final double homingRetractVoltage = -1.5;
  public static final double homingDeployVoltage = 1.5;
  public static final double homingCurrentThreshold = 15.0;

  public static final double deployedPosition = -0.023;

  public static final double retractedPosition = -3.476;

  public static final double liftPositionTolerance = 0.5;

  public static final int intakeLiftCurrentLimit = 40;
}
