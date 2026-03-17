// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import frc.robot.subsystems.intakeLift.IntakeLift;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Shooter m_shooter = new Shooter();
  private final Hopper m_hopper = new Hopper();
  private final Intake m_intake = new Intake();
  private final IntakeLift m_intakeLift = new IntakeLift();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // The operator's controller
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));
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

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // ========== OPERATOR CONTROLS ==========

    // Launch button (A) - runs shooter and hopper forward to score
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileTrue(
            new RunCommand(
                () -> {
                  m_shooter.setVelocity(ShooterConstants.shooterRPM);
                  m_hopper.setVelocity(HopperConstants.hopperFeedRPM);
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
                  m_intake.outtake();
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
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(() -> m_intake.intake(), m_intake))
        .onFalse(new InstantCommand(() -> m_intake.stop(), m_intake));

    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> m_intake.outtake(), m_intake))
        .onFalse(new InstantCommand(() -> m_intake.stop(), m_intake));

    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .onTrue(
            new ConditionalCommand(
                new RunCommand(() -> m_intakeLift.liftRetract(), m_intakeLift)
                    .until(m_intakeLift::isLiftRetracted)
                    .andThen(new InstantCommand(() -> m_intakeLift.liftStop(), m_intakeLift)),
                new RunCommand(() -> m_intakeLift.liftDeploy(), m_intakeLift)
                    .until(m_intakeLift::isLiftDeployed)
                    .andThen(new InstantCommand(() -> m_intakeLift.liftStop(), m_intakeLift)),
                m_intakeLift::isLiftDeployed));

    // Shooter only (right bumper)
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(
            new RunCommand(() -> m_shooter.setVelocity(ShooterConstants.shooterRPM), m_shooter))
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

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
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
