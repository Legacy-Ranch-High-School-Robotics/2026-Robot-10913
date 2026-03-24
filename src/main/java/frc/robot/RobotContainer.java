// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import java.util.List;

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

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // The operator's controller
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    m_fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    m_vision = new Vision(m_robotDrive, m_fieldLayout);

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
                var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
                boolean isRed =
                    alliance.isPresent()
                        && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
                edu.wpi.first.math.geometry.Translation2d targetHub =
                    isRed
                        ? frc.robot.Constants.FieldConstants.kRedHub
                        : frc.robot.Constants.FieldConstants.kBlueHub;

                edu.wpi.first.math.geometry.Rotation2d targetAngle =
                    targetHub.minus(m_robotDrive.getPose().getTranslation()).getAngle();
                rotSpeed = m_robotDrive.calculateHubTracking(targetAngle);
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

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // ========== OPERATOR CONTROLS ==========

    // Launch button (A) - runs shooter and hopper forward to score
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileTrue(
            new RunCommand(
                () -> {
                  m_shooter.setVelocity(
                      SmartDashboard.getNumber("Shooter/Target RPM", ShooterConstants.shooterRPM));
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
                        SmartDashboard.getNumber(
                            "Shooter/Target RPM", ShooterConstants.shooterRPM)),
                m_shooter))
        .onFalse(new InstantCommand(() -> m_shooter.stop(), m_shooter));
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

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
