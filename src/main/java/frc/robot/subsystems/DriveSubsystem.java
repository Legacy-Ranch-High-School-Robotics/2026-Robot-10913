// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of

// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.sim.DriveSim;
import frc.robot.telemetry.ElasticTelemetry;

public class DriveSubsystem extends SubsystemBase {
  private DriveSim m_driveSim;

  // Create MAXSwerveModules

  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor

  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kPigeon2CanId);

  // Odometry class for tracking robot pose

  SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          new Pose2d());

  // Field2d for visualizing robot pose in Glass/AdvantageScope
  private final Field2d m_field = new Field2d();

  private boolean m_isTrackingHub = false;

  // Standard PID controller handles moving setpoints much better than ProfiledPIDController
  private final PIDController m_turnToPointController = new PIDController(4.0, 0, 0);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // Usage reporting for MAXSwerve template

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // Publish the field to Elastic so Glass/Elastic dashboards can show it
    ElasticTelemetry.publishSendable("Field", m_field);

    // Configure the turn to point controller
    m_turnToPointController.enableContinuousInput(-Math.PI, Math.PI);
    m_turnToPointController.setTolerance(Math.toRadians(2.0));

    if (RobotBase.isSimulation()) {
      m_driveSim = new DriveSim(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight, m_gyro);
      // Sync odometry with the sim starting pose so robot doesn't start at (0,0)
      resetOdometry(frc.robot.sim.SimConstants.kStartingPose);
    }
  }

  /**
   * Returns the simulation wrapper for the drivetrain.
   *
   * @return The DriveSim instance, or null if not in simulation.
   */
  public DriveSim getDriveSim() {
    return m_driveSim;
  }

  @Override
  public void periodic() {

    // Update the odometry in the periodic block

    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    // Publish odometry pose to the field widget
    m_field.setRobotPose(getPose());

    // Publish Pigeon2 gyro telemetry
    ElasticTelemetry.setNumber("Gyro/Yaw", m_gyro.getYaw().getValueAsDouble());
    ElasticTelemetry.setNumber("Gyro/Pitch", m_gyro.getPitch().getValueAsDouble());
    ElasticTelemetry.setNumber("Gyro/Roll", m_gyro.getRoll().getValueAsDouble());

    publishModuleTelemetry("Drive/FrontLeft", m_frontLeft);
    publishModuleTelemetry("Drive/FrontRight", m_frontRight);
    publishModuleTelemetry("Drive/RearLeft", m_rearLeft);
    publishModuleTelemetry("Drive/RearRight", m_rearRight);

    double totalDriveCurrent =
        m_frontLeft.getDriveCurrent()
            + m_frontRight.getDriveCurrent()
            + m_rearLeft.getDriveCurrent()
            + m_rearRight.getDriveCurrent();
    ElasticTelemetry.setNumber("Drive/TotalDriveCurrent", totalDriveCurrent);
  }

  /**
   * Returns the Field2d object for visualizing the robot pose.
   *
   * @return The Field2d instance.
   */
  public Field2d getField() {
    return m_field;
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    if (RobotBase.isSimulation() && m_driveSim != null) {
      m_driveSim.setSimulationWorldPose(pose);
    }
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *     <p>field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    // Convert the commanded speeds into the correct units for the drivetrain

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;

    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;

    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, getPose().getRotation())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);

    m_frontRight.setDesiredState(swerveModuleStates[1]);

    m_rearLeft.setDesiredState(swerveModuleStates[2]);

    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setTrackingHub(boolean tracking) {
    m_isTrackingHub = tracking;
  }

  public boolean isTrackingHub() {
    return m_isTrackingHub;
  }

  /**
   * Calculates the rotational velocity ([-1, 1]) required to face a target angle.
   *
   * @param targetAngle The field-relative angle to face.
   * @return The rotational speed in [-1, 1] range for the drive() method.
   */
  public double calculateHubTracking(Rotation2d targetAngle) {
    double rotSpeed =
        m_turnToPointController.calculate(
            getPose().getRotation().getRadians(), targetAngle.getRadians());

    if (m_turnToPointController.atSetpoint()) {
      rotSpeed = 0;
    }

    // Convert from radians/sec to [-1, 1] range expected by the drive() method
    return rotSpeed / DriveConstants.kMaxAngularSpeed;
  }

  /**
   * Calculates the distance to the speaker/hub.
   *
   * @return The distance to the hub in meters.
   */
  public double getDistanceToHub() {
    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    boolean isRed =
        alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
    edu.wpi.first.math.geometry.Translation2d targetHub =
        isRed
            ? frc.robot.Constants.FieldConstants.kRedHub
            : frc.robot.Constants.FieldConstants.kBlueHub;
    return getPose().getTranslation().getDistance(targetHub);
  }

  /**
   * Calculates the target field angle to the speaker/hub.
   *
   * @return The field-relative angle from the robot to the hub.
   */
  public Rotation2d getTargetAngleToHub() {
    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    boolean isRed =
        alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
    edu.wpi.first.math.geometry.Translation2d targetHub =
        isRed
            ? frc.robot.Constants.FieldConstants.kRedHub
            : frc.robot.Constants.FieldConstants.kBlueHub;
    return targetHub.minus(getPose().getTranslation()).getAngle();
  }

  /**
   * Calculates the angle error between where the robot is currently facing and the hub. Useful for
   * telemetry to see when the robot points at the hub (error is 0).
   *
   * @return The angle error.
   */
  public Rotation2d getAngleErrorToHub() {
    return getTargetAngleToHub().minus(getPose().getRotation());
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {

    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));

    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));

    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));

    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);

    m_frontRight.setDesiredState(desiredStates[1]);

    m_rearLeft.setDesiredState(desiredStates[2]);

    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {

    m_frontLeft.resetEncoders();

    m_rearLeft.resetEncoders();

    m_frontRight.resetEncoders();

    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {

    return Rotation2d.fromDegrees(
            m_gyro.getYaw().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0))
        .getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {

    return m_gyro.getAngularVelocityZWorld().getValueAsDouble()
        * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    m_odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  public double getPitch() {
    return m_gyro.getPitch().getValueAsDouble();
  }

  public double getRoll() {
    return m_gyro.getRoll().getValueAsDouble();
  }

  private void publishModuleTelemetry(String prefix, MAXSwerveModule module) {
    ElasticTelemetry.setNumber(prefix + "/DriveCurrent", module.getDriveCurrent());
    ElasticTelemetry.setNumber(prefix + "/TurnCurrent", module.getTurnCurrent());
    ElasticTelemetry.setNumber(prefix + "/DriveTempC", module.getDriveTemperature());
    ElasticTelemetry.setNumber(prefix + "/TurnTempC", module.getTurnTemperature());
  }
}
