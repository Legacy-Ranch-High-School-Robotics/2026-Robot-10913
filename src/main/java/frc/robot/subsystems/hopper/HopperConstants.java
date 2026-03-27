package frc.robot.subsystems.hopper;

public class HopperConstants {
  public static final int hopperMotorCanId = 12;
  public static final boolean hopperMotorInverted = false;
  public static final int hopperCurrentLimit = 60;

  public static final double hopperKp = 0.0001;
  public static final double hopperKi = 0.0;
  public static final double hopperKd = 0.0;
  public static final double hopperKv = 0.0021;

  public static final double hopperIdleRPM = 1500.0;
  public static final double hopperFeedRPM = 5000.0;

  public static final double hopperToleranceRPM = 200.0;
}
