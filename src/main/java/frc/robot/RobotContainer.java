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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

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

  // The operator's controller (F310 gamepad)
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

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> {
              double xSpeed =
                  -MathUtil.applyDeadband(
                      m_driverController.getLeftY(), OIConstants.kDriveDeadband);
              double ySpeed =
                  -MathUtil.applyDeadband(
                      m_driverController.getLeftX(), OIConstants.kDriveDeadband);
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
    NamedCommands.registerCommand(
        "hopperRollers", new RunCommand(() -> m_hopper.setVoltage(6.00), m_hopper));
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

    new JoystickButton(m_driverController, XboxController.Button.kRightStick.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_robotDrive.setTrackingHub(true);
                  ElasticTelemetry.setBoolean("Drive/HubTrackingEnabled", true);
                }));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_robotDrive.setTrackingHub(false);
                  ElasticTelemetry.setBoolean("Drive/HubTrackingEnabled", false);
                }));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // ========== OPERATOR CONTROLS (F310 or Keyboard) ==========

    // Launch button (A / Space) - runs shooter and hopper forward to score
    var launchCommand =
        new RunCommand(
            () -> {
              m_shooter.setVelocity(
                  ElasticTelemetry.getNumber("Shooter/Target RPM", ShooterConstants.shooterRPM));
              if (true) {
                m_hopper.setVelocity(HopperConstants.hopperFeedRPM);
              } else {
                m_hopper.stop();
              }
            },
            m_shooter,
            m_hopper);
    var stopLaunchCommand =
        new InstantCommand(
            () -> {
              m_shooter.stop();
              m_hopper.stop();
            },
            m_shooter,
            m_hopper);

    // Eject button (B / E key) - runs intake, hopper, and shooter backwards
    var ejectCommand =
        new RunCommand(
            () -> {
              m_intake.intake();
              m_hopper.eject();
              m_shooter.eject();
            },
            m_intake,
            m_hopper,
            m_shooter);
    var stopEjectCommand =
        new InstantCommand(
            () -> {
              m_intake.stop();
              m_hopper.stop();
              m_shooter.stop();
            },
            m_intake,
            m_hopper,
            m_shooter);

    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whileTrue(ejectCommand)
        .onFalse(stopEjectCommand);

    // Intake controls
    var outtakeCommand = new RunCommand(() -> m_intake.outtake(), m_intake);
    var stopIntakeCommand = new InstantCommand(() -> m_intake.stop(), m_intake);
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(outtakeCommand)
        .onFalse(stopIntakeCommand);

    var deployCommand = new RunCommand(() -> m_intake.liftDeploy(), m_intake);
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .whileTrue(deployCommand)
        .onFalse(stopIntakeCommand);

    var retractCommand = new RunCommand(() -> m_intake.liftRetract(), m_intake);
    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .whileTrue(retractCommand)
        .onFalse(stopIntakeCommand);

    // Shooter only (right bumper / R key)
    var shooterOnlyCommand =
        new RunCommand(
            () ->
                m_shooter.setVelocity(
                    ElasticTelemetry.getNumber("Shooter/Target RPM", ShooterConstants.shooterRPM)),
            m_shooter);
    var stopShooterCommand = new InstantCommand(() -> m_shooter.stop(), m_shooter);
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(shooterOnlyCommand)
        .onFalse(stopShooterCommand);

    new POVButton(m_operatorController, 270)
        .onTrue(
            new InstantCommand(() -> setShooterPreset("Close", ShooterConstants.closePresetRPM)));

    new POVButton(m_operatorController, 0)
        .onTrue(
            new InstantCommand(
                () -> setShooterPreset("Medium Distance", ShooterConstants.atMediumPresetRPM)));

    new POVButton(m_operatorController, 90)
        .onTrue(
            new InstantCommand(
                () -> setShooterPreset("At Distance", ShooterConstants.atDistancePresetRPM)));

    new POVButton(m_operatorController, 180)
        .onTrue(
            new InstantCommand(
                () -> setShooterPreset("AirMail", ShooterConstants.airMailPresetRPM)));

    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileTrue(launchCommand)
        .onFalse(stopLaunchCommand);

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
}
