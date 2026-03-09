package frc.robot.subsystems.shooter;

public class ShooterConstants {
  //Orange Wheels
  public static final int topMotorCanId = 13;
  //Grey Wheels
  public static final int bottomMotorCanId = 12;

  public static final boolean topMotorInverted = false;
  public static final boolean bottomMotorInverted = false;

  public static final int shooterCurrentLimit = 60;

  public static final double shooterKp = 0.0001;
  public static final double shooterKi = 0.0;
  public static final double shooterKd = 0.0;
  public static final double shooterKv = 0.000175;

  public static final double shooterIdleRPM = 1500.0;
  public static final double shooterSpeakerRPM = 5650.0;
  public static final double shooterAmpRPM = 2500.0;

  public static final double shooterToleranceRPM = 3000.0;
}
