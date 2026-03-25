package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

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

  public static final double speakerPresetRPM = 4600.0;
  public static final double ampPresetRPM = 3000.0;
  public static final double trapPresetRPM = 5200.0;

  public static final double shooterToleranceRPM = 150.0;

  // Interpolation map for distance in meters (converted from feet) to RPM
  public static final InterpolatingDoubleTreeMap distanceToRpmMap =
      new InterpolatingDoubleTreeMap();

  static {
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(2.0), 2000.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(3.0), 2500.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(5.0), 3500.0);
    distanceToRpmMap.put(edu.wpi.first.math.util.Units.feetToMeters(7.0), 5000.0);
  }
}
