package frc.robot.subsystems.shooter;

public class ShooterConstants {
  // Orange Wheels
  public static final int topMotorCanId = 13;

  public static final boolean topMotorInverted = false;

  public static final int shooterCurrentLimit = 60;

  public static final double shooterKp = 0.0001;
  public static final double shooterKi = 0.0;
  public static final double shooterKd = 0.0;
  public static final double shooterKv = 0.000175;

  public static final double shooterRPM = 4575.0;
  public static final double shooterRPMInverted = -2000.0;

  public static final double shooterToleranceRPM = 100.0;
}
