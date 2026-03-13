// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Shooter m_shooter = new Shooter();
  private final Hopper m_hopper = new Hopper();
  private final Intake m_intake = new Intake();

  // Autonomous Chooser
  private final SendableChooser<Command> autoChooser;

  // Controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();  

    // Build PathPlanner Auto Chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Default drive command
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  private void configureButtonBindings() {
    // ========== DRIVER CONTROLS ==========
    // X-Stance
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(m_robotDrive::setX, m_robotDrive));

    // Reset Gyro
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    // ========== OPERATOR CONTROLS ==========
    
    // Combined A Button: Runs both Shooter and Hopper
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(() -> {
            m_shooter.setVelocity(ShooterConstants.shooterRPM);
            m_hopper.setVelocity(ShooterConstants.shooterRPM);
        }, m_shooter, m_hopper))
        .onFalse(new InstantCommand(() -> {
            m_shooter.stop();
            m_hopper.stop();
        }, m_shooter, m_hopper));

    // Eject (B)
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> {
              m_intake.outtake();
              m_hopper.eject();
              m_shooter.eject();
            }, m_intake, m_hopper, m_shooter))
        .onFalse(new InstantCommand(() -> {
              m_intake.stop();
              m_hopper.stop();
              m_shooter.stop();
            }, m_intake, m_hopper, m_shooter));

    // Intake (X)
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(m_intake::intake, m_intake))
        .onFalse(new InstantCommand(m_intake::stop, m_intake));

    // Outtake (Y)
    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(m_intake::outtake, m_intake))
        .onFalse(new InstantCommand(m_intake::stop, m_intake));

    // Intake Lift (Left Bumper)
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(m_intake::liftDeploy, m_intake).until(m_intake::isLiftDeployed))
        .onFalse(new InstantCommand(m_intake::liftStop, m_intake));
  };

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
