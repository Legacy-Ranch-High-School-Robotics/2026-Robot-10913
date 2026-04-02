// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.SpinUpShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.telemetry.ElasticTelemetry;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Update these IDs to the AprilTags that correspond to the trench on your field.
  private static final int[] TRENCH_TAG_IDS = {7, 6, 12, 1, 17, 28, 22, 23};
  private static final double SLOW_TRANSLATION_SCALE =
      1.5 / DriveConstants.kMaxSpeedMetersPerSecond;
  private static final double MEDIUM_TRANSLATION_SCALE =
      3.0 / DriveConstants.kMaxSpeedMetersPerSecond;
  private static final double FAST_TRANSLATION_SCALE = 1.0;

  // The robot's subsystems
  private final AprilTagFieldLayout m_fieldLayout;
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Vision m_vision;
  private final Shooter m_shooter = new Shooter();
  private final Hopper m_hopper = new Hopper();
  private final Intake m_intake = new Intake();
  private final SendableChooser<Command> m_autoChooser;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private double m_driverTranslationScale = MEDIUM_TRANSLATION_SCALE;

  // The operator's controller
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register PathPlanner named commands and configure auto builder/logging
    configurePathPlannerCommands();
    configureAutoBuilder();
    configureAutoLogging();
    if (AutoBuilder.isConfigured()) {
      m_autoChooser = AutoBuilder.buildAutoChooser("Auton1");
    } else {
      m_autoChooser = new SendableChooser<>();
      m_autoChooser.setDefaultOption("None", Commands.none());
      DriverStation.reportError(
          "AutoBuilder not configured. Did you generate settings.json in PathPlanner?", false);
    }
    ElasticTelemetry.publishSendable("Auto/Chooser", m_autoChooser);
    m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    m_fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    m_vision = new Vision(m_robotDrive, m_fieldLayout);
    m_shooter.setDistanceSupplier(m_robotDrive::getDistanceToHub);
    m_shooter.setAngleSupplier(m_robotDrive::getAngleErrorToHub);
    setDriverSpeedPreset("Medium", MEDIUM_TRANSLATION_SCALE);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> {
              double xSpeed =
                  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)
                      * m_driverTranslationScale;
              double ySpeed =
                  -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)
                      * m_driverTranslationScale;
              double rotSpeed =
                  -MathUtil.applyDeadband(
                      m_driverController.getRightX(), OIConstants.kDriveDeadband);

              if (m_robotDrive.isTrackingHub()) {
                rotSpeed = m_robotDrive.calculateHubTracking(m_robotDrive.getTargetAngleToHub());
              }

              m_robotDrive.drive(xSpeed, ySpeed, rotSpeed, true);
            },
            m_robotDrive));
  }

  /**
   * Returns the drive subsystem.
   *
   * @return The drive subsystem.
   */
  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  /** Registers commands with PathPlanner for use in event markers. */
  private void configurePathPlannerCommands() {
    // Shooter commands
    NamedCommands.registerCommand("SpinUpShooter", new SpinUpShooter(m_shooter));
    NamedCommands.registerCommand("Shoot", new ShootCommand(m_shooter, m_hopper));
    NamedCommands.registerCommand("StopShooter", new StopShooter(m_shooter));

    // Intake commands
    NamedCommands.registerCommand("DeployIntake", new DeployIntake(m_intake));
    NamedCommands.registerCommand("Intake", new IntakeCommand(m_intake));
    NamedCommands.registerCommand("RetractIntake", new RetractIntake(m_intake));
    NamedCommands.registerCommand("StopIntake", new StopIntake(m_intake));

    // Hopper utility command for PathPlanner markers
    NamedCommands.registerCommand(
        "HoppperRollers", new InstantCommand(() -> m_hopper.setVelocity(6), m_hopper));
  }

  public AprilTagFieldLayout getFieldLayout() {
    return m_fieldLayout;
  }

  public Vision getVision() {
    return m_vision;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * JoystickButton}.
   */
  private void configureButtonBindings() {

    // ========== DRIVER CONTROLS ==========

    // Drive controls
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // Toggle Hub Tracking
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(
            new InstantCommand(() -> m_robotDrive.setTrackingHub(!m_robotDrive.isTrackingHub())));

    // Press A to rotate in place toward the nearest trench AprilTag, then stop
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(rotateToNearestTrenchTagCommand());

    // Driver speed presets (easy-to-reach face buttons)
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(() -> setDriverSpeedPreset("Slow", SLOW_TRANSLATION_SCALE)));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> setDriverSpeedPreset("Medium", MEDIUM_TRANSLATION_SCALE)));
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(() -> setDriverSpeedPreset("Fast", FAST_TRANSLATION_SCALE)));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // ========== OPERATOR CONTROLS ==========

    // Launch button (A) - runs shooter and hopper forward to score
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileTrue(
            new RunCommand(
                () -> {
                  m_shooter.setVelocity(
                      ElasticTelemetry.getNumber(
                          "Shooter/Target RPM", ShooterConstants.shooterRPM));
                  // Set Target Rpm in the method below
                  if (m_shooter.atTargetVelocity()) {
                    m_hopper.setVelocity(HopperConstants.hopperFeedRPM);
                  } else {
                    m_hopper.stop();
                  }
                },
                m_shooter,
                m_hopper))
        .onFalse(
            new InstantCommand(
                () -> {
                  m_shooter.stop();
                  m_hopper.stop();
                },
                m_shooter,
                m_hopper));

    // Eject button (B) - runs intake, hopper, and shooter backwards
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whileTrue(
            new RunCommand(
                () -> {
                  m_intake.intake();
                  m_hopper.eject();
                  m_shooter.eject();
                },
                m_intake,
                m_hopper,
                m_shooter))
        .onFalse(
            new InstantCommand(
                () -> {
                  m_intake.stop();
                  m_hopper.stop();
                  m_shooter.stop();
                },
                m_intake,
                m_hopper,
                m_shooter));

    // Intake controls
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> m_intake.outtake(), m_intake))
        .onFalse(new InstantCommand(() -> m_intake.stop(), m_intake));

    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(() -> m_intake.liftDeploy(), m_intake))
        .onFalse(new InstantCommand(() -> m_intake.stop(), m_intake));

    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> m_intake.liftRetract(), m_intake))
        .onFalse(new InstantCommand(() -> m_intake.stop(), m_intake));

    // Shooter only (right bumper)
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(
            new RunCommand(
                () ->
                    m_shooter.setVelocity(
                        ElasticTelemetry.getNumber(
                            "Shooter/Target RPM", ShooterConstants.shooterRPM)),
                m_shooter))
        .onFalse(new InstantCommand(() -> m_shooter.stop(), m_shooter));

    // Shooter presets on D-pad
    new POVButton(m_operatorController, 0)
        .onTrue(
            new InstantCommand(
                () -> setShooterPreset("Speaker", ShooterConstants.speakerPresetRPM)));
    new POVButton(m_operatorController, 90)
        .onTrue(new InstantCommand(() -> setShooterPreset("Amp", ShooterConstants.ampPresetRPM)));
    new POVButton(m_operatorController, 180)
        .onTrue(new InstantCommand(() -> setShooterPreset("Trap", ShooterConstants.trapPresetRPM)));

    // Operator rumble feedback when shooter is ready
    new Trigger(m_shooter::atTargetVelocity)
        .onTrue(
            new InstantCommand(() -> m_operatorController.setRumble(RumbleType.kBothRumble, 1.0)))
        .onFalse(
            new InstantCommand(() -> m_operatorController.setRumble(RumbleType.kBothRumble, 0.0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // Determine starting pose based on DriverStation alliance and station
    Pose2d startPose = frc.robot.sim.SimConstants.getStartingPose();
    double dir = startPose.getRotation().getDegrees() == 180 ? -1 : 1;
    double x = startPose.getX();
    double y = startPose.getY();
    Rotation2d rot = startPose.getRotation();

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            startPose,
            List.of(new Translation2d(x + 1 * dir, y + 1), new Translation2d(x + 2 * dir, y - 1)),
            new Pose2d(x + 3 * dir, y, rot),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    Command auto = m_autoChooser.getSelected();
    return auto != null ? auto : Commands.none();
  }

  private void setShooterPreset(String presetName, double rpm) {
    ElasticTelemetry.setNumber("Shooter/Target RPM", rpm);
    ElasticTelemetry.setString("Shooter/ActivePreset", presetName);
  }

  private void configureAutoBuilder() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }

    AutoBuilder.configure(
        m_robotDrive::getPose,
        m_robotDrive::resetOdometry,
        m_robotDrive::getChassisSpeeds,
        m_robotDrive::driveRobotRelative,
        new PPHolonomicDriveController(
            new PIDConstants(AutoConstants.kPXController, 0, 0),
            new PIDConstants(AutoConstants.kPThetaController, 0, 0)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        },
        m_robotDrive);
  }

  private void configureAutoLogging() {
    PathPlannerLogging.setLogActivePathCallback(
        poses -> m_robotDrive.getField().getObject("Auto/PlannedPath").setPoses(poses));
    PathPlannerLogging.setLogCurrentPoseCallback(
        pose -> m_robotDrive.getField().getObject("Auto/CurrentPose").setPose(pose));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> m_robotDrive.getField().getObject("Auto/TargetPose").setPose(pose));
  }

  private Rotation2d getNearestTrenchTagAngle() {
    Translation2d robotTranslation = m_robotDrive.getPose().getTranslation();
    Optional<Pose2d> nearestTagPose =
        m_fieldLayout.getTags().stream()
            .filter(tag -> Arrays.stream(TRENCH_TAG_IDS).anyMatch(id -> id == tag.ID))
            .map(tag -> tag.pose.toPose2d())
            .min(
                (a, b) ->
                    Double.compare(
                        robotTranslation.getDistance(a.getTranslation()),
                        robotTranslation.getDistance(b.getTranslation())));

    if (nearestTagPose.isEmpty()) {
      return null;
    }

    return nearestTagPose.get().getTranslation().minus(robotTranslation).getAngle();
  }

  private Command rotateToNearestTrenchTagCommand() {
    final Rotation2d[] targetAngle = new Rotation2d[1];
    return new FunctionalCommand(
        () -> targetAngle[0] = getNearestTrenchTagAngle(),
        () -> {
          if (targetAngle[0] == null) {
            return;
          }
          double rotSpeed = m_robotDrive.calculateHubTracking(targetAngle[0]);
          m_robotDrive.drive(0.0, 0.0, rotSpeed, true);
        },
        interrupted -> m_robotDrive.drive(0.0, 0.0, 0.0, true),
        () ->
            targetAngle[0] == null
                || Math.abs(targetAngle[0].minus(m_robotDrive.getPose().getRotation()).getDegrees())
                    < 2.0,
        m_robotDrive);
  }

  private void setDriverSpeedPreset(String presetName, double translationScale) {
    m_driverTranslationScale = translationScale;
    ElasticTelemetry.setString("Drive/SpeedPreset", presetName);
    ElasticTelemetry.setNumber(
        "Drive/SpeedPresetMetersPerSecond",
        DriveConstants.kMaxSpeedMetersPerSecond * translationScale);
  }
}
