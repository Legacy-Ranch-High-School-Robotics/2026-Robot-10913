package frc.robot.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.photonvision.simulation.SimCameraProperties;

/**
 * Constants specific to the simulation environment. All physical properties used by maple-sim are
 * defined here.
 */
public final class SimConstants {

  // Robot physical properties
  public static final Mass kRobotMass = Kilograms.of(54.0); // ~120 lbs
  public static final Distance kBumperLengthX = Meters.of(0.813); // ~32 inches
  public static final Distance kBumperWidthY = Meters.of(0.813); // ~32 inches
  public static final Distance kTrackLength = Meters.of(0.673); // 26.5 inches
  public static final Distance kTrackWidth = Meters.of(0.673); // 26.5 inches

  // FRC field dimensions (meters)
  public static final double kFieldLengthX = frc.robot.Constants.FieldConstants.kFieldLength;
  public static final double kFieldWidth = frc.robot.Constants.FieldConstants.kFieldWidth;

  // Default starting pose (used before DriverStation data is available)
  public static final Pose2d kStartingPose = new Pose2d(2.0, 6.5, new Rotation2d());

  /**
   * Computes a starting pose based on the current DriverStation alliance and station number. Places
   * the robot 1.5m from the alliance wall, spaced 1m apart per station (y = 4.5, 5.5, 6.5). Falls
   * back to {@link #kStartingPose} if alliance data is not yet available.
   */
  public static Pose2d getStartingPose() {
    var alliance = DriverStation.getAlliance();
    int station = DriverStation.getLocation().orElse(1);

    double distance = 1.5;

    double x = kStartingPose.getX();
    double y = (kFieldWidth / 2) + (station - 2) * distance;
    Rotation2d rot = new Rotation2d(0);

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      x = kFieldLengthX - kStartingPose.getX();
      rot = Rotation2d.fromDegrees(180);
    }

    return new Pose2d(x, y, rot);
  }

  /**
   * Creates a fully configured DriveTrainSimulationConfig for our robot. Uses NEO Vortex drive
   * motors and NEO 550 steering motors (same as our MAXSwerve modules) and MAXSwerve gearing.
   */
  public static DriveTrainSimulationConfig createDriveTrainConfig() {
    return new DriveTrainSimulationConfig(
        kRobotMass,
        kBumperLengthX,
        kBumperWidthY,
        kTrackLength,
        kTrackWidth,
        COTS.ofPigeon2(), // Gyro sim (closest available; we use ADIS16470 on real robot)
        COTS.ofMAXSwerve(
            DCMotor.getNeoVortex(1), // Drive motor: NEO Vortex
            DCMotor.getNeo550(1), // Steer motor: NEO 550
            COTS.WHEELS.COLSONS.cof, // Colson wheels COF
            3 // MAXSwerve Base Kit L3 = 14T pinion
            ));
  }

  /** Gets the global simulation properties for the cameras. */
  public static SimCameraProperties createCameraProperties() {
    SimCameraProperties properties = new SimCameraProperties();
    properties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    properties.setCalibError(0.35, 0.10);
    properties.setFPS(15);
    properties.setAvgLatencyMs(50);
    properties.setLatencyStdDevMs(15);
    return properties;
  }

  // ===== Mechanism Simulation Constants =====

  // Shooter flywheel (SparkFlex + NEO Vortex, CAN 13)
  public static final double kShooterFlywheelMOI = 0.004; // kg·m²
  public static final double kShooterGearing =
      frc.robot.subsystems.shooter.ShooterConstants.shooterGearRatio;

  // Hopper feed wheel (SparkMax + NEO, CAN 12)
  public static final double kHopperMOI = 0.001; // kg·m²

  // Intake roller (SparkMax + NEO 550, CAN 11)
  public static final double kIntakeRollerMOI = 0.001; // kg·m²

  // Intake arm / lift (SparkMax + NEO 550, CAN 10)
  // Belt-driven arm: 3.476 motor rotations sweeps the arm π/2 radians (0° to 90°)
  public static final double kIntakeArmLengthMeters = 0.3;
  public static final double kIntakeArmMassKg = 1.5;
  public static final double kIntakeArmGearing =
      frc.robot.subsystems.intake.IntakeConstants.intakeLiftGearRatio;
  public static final double kIntakeArmMinAngleRad = 0.0; // horizontal (deployed)
  public static final double kIntakeArmMaxAngleRad = Math.PI / 2.0; // vertical (retracted)
  public static final double kIntakeArmStartingAngleRad = Math.PI / 2.0; // starts retracted
}
