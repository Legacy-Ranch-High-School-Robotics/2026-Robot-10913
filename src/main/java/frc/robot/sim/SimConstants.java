package frc.robot.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/**
 * Constants specific to the simulation environment.
 * All physical properties used by maple-sim are defined here.
 */
public final class SimConstants {

    // Robot physical properties
    public static final Mass kRobotMass = Kilograms.of(54.0); // ~120 lbs
    public static final Distance kBumperLengthX = Meters.of(0.813); // ~32 inches
    public static final Distance kBumperWidthY = Meters.of(0.813);  // ~32 inches
    public static final Distance kTrackLength = Meters.of(0.673);   // 26.5 inches
    public static final Distance kTrackWidth = Meters.of(0.673);    // 26.5 inches

    // Starting pose for simulation (open area on blue alliance side, facing right)
    // FRC field: 16.54m x 8.21m. Avoiding the cage/hanger in the center.
    public static final Pose2d kStartingPose = new Pose2d(1.5, 6.5, new Rotation2d());

    /**
     * Creates a fully configured DriveTrainSimulationConfig for our robot.
     * Uses NEO motors (same as our MAXSwerve modules) and MAXSwerve gearing.
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
                DCMotor.getNEO(1),       // Drive motor: NEO
                DCMotor.getNeo550(1),     // Steer motor: NEO 550
                COTS.WHEELS.COLSONS.cof,  // Colson wheels COF
                3                         // MAXSwerve Base Kit L3 = 14T pinion
            )
        );
    }
}
