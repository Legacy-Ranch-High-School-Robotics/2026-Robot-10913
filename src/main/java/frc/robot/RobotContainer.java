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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.hopper.Hopper;
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

  // Speed preset state — Normal is the default
  private double m_driveMaxSpeed = DriveConstants.kNormalSpeedMetersPerSecond;
  private double m_driveMaxAngularSpeed = DriveConstants.kNormalAngularSpeed;
  private String m_speedModeName = "Normal";

  private final RobotCommands.ShootOnMove m_shootOnMoveCommand =
      new RobotCommands.ShootOnMove(m_shooter, m_hopper, m_robotDrive);

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
    ElasticTelemetry.setString("Drive/ShootingPreset", "Hub");

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
              } else {
                rotSpeed *= m_driveMaxAngularSpeed / DriveConstants.kMaxAngularSpeed;
              }

              double scaledX = xSpeed * m_driveMaxSpeed / DriveConstants.kMaxSpeedMetersPerSecond;
              double scaledY = ySpeed * m_driveMaxSpeed / DriveConstants.kMaxSpeedMetersPerSecond;

              ElasticTelemetry.setNumber("Drive/MaxSpeed", m_driveMaxSpeed);
              ElasticTelemetry.setNumber("Drive/MaxAngularSpeed", m_driveMaxAngularSpeed);
              ElasticTelemetry.setString("Drive/SpeedMode", m_speedModeName);
              ElasticTelemetry.setNumber("Drive/Joystick/X", scaledX);
              ElasticTelemetry.setNumber("Drive/Joystick/Y", scaledY);
              ElasticTelemetry.setNumber("Drive/Joystick/Rot", rotSpeed);
              ElasticTelemetry.setBoolean("Drive/HubTrackingEnabled", m_robotDrive.isTrackingHub());

              m_robotDrive.drive(scaledX, scaledY, rotSpeed, true);
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
    NamedCommands.registerCommand("SpinUpShooter", RobotCommands.spinUpShooter(m_shooter));
    NamedCommands.registerCommand("Shoot", RobotCommands.shoot(m_shooter, m_hopper));
    NamedCommands.registerCommand("StopShooter", RobotCommands.stopShooter(m_shooter));

    // Intake commands
    NamedCommands.registerCommand("DeployIntake", RobotCommands.deployIntake(m_intake));
    NamedCommands.registerCommand("Intake", RobotCommands.intakeRollers(m_intake));
    NamedCommands.registerCommand("RetractIntake", RobotCommands.retractIntake(m_intake));
    NamedCommands.registerCommand("StopIntake", RobotCommands.stopIntake(m_intake));
    NamedCommands.registerCommand(
        "hopperRollers", new RunCommand(() -> m_hopper.setVoltage(6.00), m_hopper));
  }

  public AprilTagFieldLayout getFieldLayout() {
    return m_fieldLayout;
  }

  public Vision getVision() {
    return m_vision;
  }

  public Shooter getShooter() {
    return m_shooter;
  }

  public Hopper getHopper() {
    return m_hopper;
  }

  public Intake getIntake() {
    return m_intake;
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
                  m_shootOnMoveCommand.schedule();
                  ElasticTelemetry.setBoolean("Drive/HubTrackingEnabled", true);
                }));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_shootOnMoveCommand.cancel();
                  m_robotDrive.setTrackingHub(false);
                  m_robotDrive.zeroHeading();
                  ElasticTelemetry.setBoolean("Drive/HubTrackingEnabled", false);
                }));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // Driver D-Pad: speed presets (↑=Turbo, →=Fast, ↓=Normal, ←=Slow)
    new POVButton(m_driverController, 0)
        .onTrue(
            new InstantCommand(
                () ->
                    setSpeedMode(
                        "Turbo",
                        DriveConstants.kTurboSpeedMetersPerSecond,
                        DriveConstants.kTurboAngularSpeed)));
    new POVButton(m_driverController, 90)
        .onTrue(
            new InstantCommand(
                () ->
                    setSpeedMode(
                        "Fast",
                        DriveConstants.kFastSpeedMetersPerSecond,
                        DriveConstants.kFastAngularSpeed)));
    new POVButton(m_driverController, 180)
        .onTrue(
            new InstantCommand(
                () ->
                    setSpeedMode(
                        "Normal",
                        DriveConstants.kNormalSpeedMetersPerSecond,
                        DriveConstants.kNormalAngularSpeed)));
    new POVButton(m_driverController, 270)
        .onTrue(
            new InstantCommand(
                () ->
                    setSpeedMode(
                        "Slow",
                        DriveConstants.kSlowSpeedMetersPerSecond,
                        DriveConstants.kSlowAngularSpeed)));

    // ========== OPERATOR CONTROLS (F310 or Keyboard) ==========

    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileTrue(RobotCommands.launch(m_shooter, m_hopper));

    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whileTrue(RobotCommands.eject(m_intake, m_hopper, m_shooter));

    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(RobotCommands.outtakeRollers(m_intake));

    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .whileTrue(RobotCommands.deployIntake(m_intake));

    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .whileTrue(RobotCommands.retractIntake(m_intake));

    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(RobotCommands.spinUpAndHold(m_shooter));

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

  private void setSpeedMode(String name, double maxSpeed, double maxAngularSpeed) {
    m_driveMaxSpeed = maxSpeed;
    m_driveMaxAngularSpeed = maxAngularSpeed;
    m_speedModeName = name;
    ElasticTelemetry.setNumber("Drive/MaxSpeed", maxSpeed);
    ElasticTelemetry.setString("Drive/SpeedMode", name);
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
