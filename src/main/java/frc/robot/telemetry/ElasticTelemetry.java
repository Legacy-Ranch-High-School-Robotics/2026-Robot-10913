package frc.robot.telemetry;

import edu.wpi.first.networktables.GenericEntry;
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

  public static GenericEntry getEntry(String key) {
    return ROOT_TABLE.getEntry(key);
  }

  public static void publishSendable(String key, Sendable data) {
    SmartDashboard.putData(key, data);
  }
}
