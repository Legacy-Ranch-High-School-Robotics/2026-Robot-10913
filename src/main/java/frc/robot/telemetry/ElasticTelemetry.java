package frc.robot.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Utility class for publishing data to the Elastic dashboard via NetworkTables. */
public final class ElasticTelemetry {
  private static final NetworkTable ROOT_TABLE =
      NetworkTableInstance.getDefault().getTable("SmartDashboard");

  private ElasticTelemetry() {}

  public static void setNumber(String key, double value) {
    ROOT_TABLE.getEntry(key).setDouble(value);
  }

  public static double getNumber(String key, double defaultValue) {
    return ROOT_TABLE.getEntry(key).getDouble(defaultValue);
  }

  public static void setString(String key, String value) {
    ROOT_TABLE.getEntry(key).setString(value);
  }

  public static void setBoolean(String key, boolean value) {
    ROOT_TABLE.getEntry(key).setBoolean(value);
  }

  public static void setInteger(String key, long value) {
    ROOT_TABLE.getEntry(key).setInteger(value);
  }

  public static edu.wpi.first.networktables.NetworkTableEntry getEntry(String key) {
    return ROOT_TABLE.getEntry(key);
  }

  public static void publishSendable(String key, Sendable data) {
    SmartDashboard.putData(key, data);
  }

  public static void setPose(String key, edu.wpi.first.math.geometry.Pose2d pose) {
    ROOT_TABLE.getEntry(key).setDoubleArray(
        new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});
  }

  public static void setPoseArray(String key, edu.wpi.first.math.geometry.Pose2d[] poses) {
    double[] poseData = new double[poses.length * 3];
    for (int i = 0; i < poses.length; i++) {
      poseData[i * 3] = poses[i].getX();
      poseData[i * 3 + 1] = poses[i].getY();
      poseData[i * 3 + 2] = poses[i].getRotation().getRadians();
    }
    ROOT_TABLE.getEntry(key).setDoubleArray(poseData);
  }
}
