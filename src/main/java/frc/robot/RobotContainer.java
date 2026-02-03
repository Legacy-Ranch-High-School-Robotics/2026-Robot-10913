// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<DriverStation.Alliance> allianceChooser;
  private final LoggedDashboardChooser<Constants.StartingPosition> positionChooser;

  // Field settings
  private final boolean enableRampCollider = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(0, 0, new Rotation2d()));
        // Note: Field reset is handled in constructor for proper initial state
        // add the simulated drivetrain to the simulation field
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(enableRampCollider));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // Set up choosers
    allianceChooser = new LoggedDashboardChooser<>("Starting Alliance");
    allianceChooser.addDefaultOption("Blue", DriverStation.Alliance.Blue);
    allianceChooser.addOption("Red", DriverStation.Alliance.Red);

    positionChooser = new LoggedDashboardChooser<>("Starting Position");
    positionChooser.addDefaultOption("POS1", Constants.StartingPosition.POS1);
    positionChooser.addOption("POS2", Constants.StartingPosition.POS2);
    positionChooser.addOption("POS3", Constants.StartingPosition.POS3);

    // Initialize field to default selection
    if (Constants.currentMode == Constants.Mode.SIM) {
      resetSimulationField();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getHID().getRawAxis(2)));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> {
              resetSimulationField();
              drive.setPose(driveSimulation.getSimulatedDriveTrainPose());
            }
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)); // zero gyro
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Pose2d startingPose = getSimulationStartingPose();
    driveSimulation.setSimulationWorldPose(startingPose);
    drive.setPose(startingPose);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public Pose2d getSimulationStartingPose() {
    DriverStation.Alliance alliance = allianceChooser.get();
    if (alliance == null) alliance = DriverStation.Alliance.Blue;

    Constants.StartingPosition position = positionChooser.get();
    if (position == null) position = Constants.StartingPosition.POS1;

    Pose2d bluePose = DriveConstants.kBlueStartingPoses.get(position);

    if (alliance == DriverStation.Alliance.Red) {
      // Mirror for Red alliance
      // Assuming field length is approx 16.54m (standard FRC field)
      // Or use a utility if available. For now, mirroring X and Angle.
      double fieldLength = 16.54175; // 2024 field length
      Translation2d redTranslation =
          new Translation2d(fieldLength - bluePose.getX(), bluePose.getY());
      Rotation2d redRotation = bluePose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
      return new Pose2d(redTranslation, redRotation);
    }

    return bluePose;
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    // Sync Alliance Chooser to DriverStationSim
    DriverStation.Alliance alliance = allianceChooser.get();
    if (alliance != null) {
      if (alliance == DriverStation.Alliance.Blue) {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      } else {
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
      }
    }

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPose", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/RobotPose3d", new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}
