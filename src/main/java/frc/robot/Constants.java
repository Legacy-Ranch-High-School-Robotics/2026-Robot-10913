// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 2.0; // DEBUG:DRIVE_MAX_SPEED - Maximum drive speed (m/s)
    public static final double kMaxAngularSpeed = 2 * Math.PI; // DEBUG:DRIVE_MAX_ANGULAR_SPEED - Maximum rotation speed (rad/s)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5); // DEBUG:DRIVE_TRACK_WIDTH - Left-to-right wheel distance
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5); // DEBUG:DRIVE_WHEEL_BASE - Front-to-back wheel distance
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0;

    public static final double kDriveBaseRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);

    // SPARK MAX CAN IDs - Drive motors (SparkFlex)
    public static final int kFrontLeftDrivingCanId = 5;   // DEBUG:CAN_FL_DRIVE - FL drive motor
    public static final int kRearLeftDrivingCanId = 7;    // DEBUG:CAN_RL_DRIVE - RL drive motor
    public static final int kFrontRightDrivingCanId = 3;  // DEBUG:CAN_FR_DRIVE - FR drive motor
    public static final int kRearRightDrivingCanId = 1;   // DEBUG:CAN_RR_DRIVE - RR drive motor

    // SPARK MAX CAN IDs - Turning motors (SparkMax)
    public static final int kFrontLeftTurningCanId = 6;   // DEBUG:CAN_FL_TURN - FL turn motor
    public static final int kRearLeftTurningCanId = 8;    // DEBUG:CAN_RL_TURN - RL turn motor
    public static final int kFrontRightTurningCanId = 4;  // DEBUG:CAN_FR_TURN - FR turn motor
    public static final int kRearRightTurningCanId = 2;   // DEBUG:CAN_RR_TURN - RR turn motor

    public static final int kPigeon2CanId = 9;            // DEBUG:CAN_GYRO - Gyro (Pigeon 2.0)

    public static final boolean kGyroReversed = false;
  }

  public static final class FieldConstants {
    public static final double kFieldLength = 16.54;  // DEBUG:FIELD_LENGTH - Field length (meters)
    public static final double kFieldWidth = 8.07;    // DEBUG:FIELD_WIDTH - Field width (meters)
    public static final Translation2d kBlueHub = new Translation2d(4.63, kFieldWidth / 2.0);  // DEBUG:BLUE_HUB_POSITION - Blue speaker position
    public static final Translation2d kRedHub =
        new Translation2d(kFieldLength - 4.55, kFieldWidth / 2.0);  // DEBUG:RED_HUB_POSITION - Red speaker position
  }
  // Remember To Edit when changing gears 
  public static final class ModuleConstants {
   
    public static final int kDrivingMotorPinionTeeth = 14;  // DEBUG:PINION_TEETH - Pinion gear teeth count

    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;  // DEBUG:MOTOR_FREE_SPEED - Motor free speed (RPS)
    public static final double kWheelDiameterMeters = 0.0762;  // DEBUG:WHEEL_DIAMETER - Wheel diameter (3 inches)
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;  // DEBUG:WHEEL_CIRCUMFERENCE - Wheel circumference
    
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);  // DEBUG:GEAR_REDUCTION - Gear reduction ratio
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;  // DEBUG:WHEEL_MAX_SPEED - Max wheel speed
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;      // DEBUG:DRIVER_PORT - Driver Xbox controller USB port
    public static final int kOperatorControllerPort = 1;    // DEBUG:OPERATOR_PORT - Operator F310 controller USB port
    public static final int kKeyboardPort = 2;              // DEBUG:KEYBOARD_PORT - Keyboard USB port (not currently used)
    public static final double kDriveDeadband = 0.5;        // DEBUG:DRIVE_DEADBAND - Joystick deadband (0.0-1.0)
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.2;                          // DEBUG:AUTO_MAX_SPEED - Auto max speed (m/s)
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;              // DEBUG:AUTO_MAX_ACCEL - Auto max acceleration
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;              // DEBUG:AUTO_MAX_ANGULAR_SPEED - Auto max rotation speed
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;       // DEBUG:AUTO_MAX_ANGULAR_ACCEL - Auto max rotation accel

    public static final double kPXController = 1;          // DEBUG:AUTO_PID_X - X-axis PID proportional gain
    public static final double kPYController = 1;          // DEBUG:AUTO_PID_Y - Y-axis PID proportional gain
    public static final double kPThetaController = 1;      // DEBUG:AUTO_PID_THETA - Rotation PID proportional gain

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;  // DEBUG:NEO_FREE_SPEED - NEO motor free speed (RPM)
  }

  // ========================================
  // DEBUGGING GUIDE - QUICK REFERENCE
  // ========================================
  // Use this section to quickly identify CAN IDs and important values during debugging.
  // All values are defined in their respective constant classes above.
  //
  // === CAN ID ASSIGNMENTS ===
  // 
  // DRIVETRAIN (8 motors + 1 gyro = 9 devices):
  //   CAN 1  - Rear Right Drive (SparkFlex)
  //   CAN 2  - Rear Right Turn (SparkMax)
  //   CAN 3  - Front Right Drive (SparkFlex)
  //   CAN 4  - Front Right Turn (SparkMax)
  //   CAN 5  - Front Left Drive (SparkFlex)
  //   CAN 6  - Front Left Turn (SparkMax)
  //   CAN 7  - Rear Left Drive (SparkFlex)
  //   CAN 8  - Rear Left Turn (SparkMax)
  //   CAN 9  - Pigeon 2.0 Gyro
  //
  // INTAKE (2 motors):
  //   CAN 10 - Intake Lift Motor (SparkMax NEO550) - See IntakeConstants.java
  //   CAN 11 - Intake Roller Motor (SparkMax NEO550) - See IntakeConstants.java
  //
  // HOPPER (1 motor):
  //   CAN 12 - Hopper Motor (SparkMax NEO) - See HopperConstants.java
  //
  // SHOOTER (1 motor):
  //   CAN 13 - Shooter Top Motor (SparkFlex NEO Vortex) - See ShooterConstants.java
  //
  // === IMPORTANT SUBSYSTEM VALUES ===
  //
  // SHOOTER:
  //   - Gear Ratio: 2:1 (motor @ 4000 RPM → flywheel @ 2000 RPM)
  //   - Velocity Conversion Factor: 0.5
  //   - Close Preset: 3000 RPM (flywheel speed)
  //   - At Distance Preset: 5200 RPM (flywheel speed)
  //   - Tolerance: ±150 RPM
  //   - PID: Kp=0.0001, Ki=0.0, Kd=0.0, Kv=0.0021
  //
  // HOPPER:
  //   - Idle RPM: 1500
  //   - Feed RPM: 5000
  //   - Tolerance: ±200 RPM
  //   - PID: Kp=0.0001, Ki=0.0, Kd=0.0, Kv=0.0021
  //
  // INTAKE:
  //   - Deployed Position: 3.476 rotations
  //   - Retracted Position: 0 rotations
  //   - Upper Soft Limit: 4.0 rotations (NEED TO TEST!!!!)
  //   - Lower Soft Limit: -0.5 rotations (NEED TO TEST!!!!)
  //   - Position Tolerance: ±0.5 rotations
  //   - Intake Voltage: 10.0V
  //   - Outtake Voltage: -8.0V
  //   - Lift Voltage: 1.25V
  //
  // DRIVETRAIN:
  //   - Max Speed: 2.0 m/s
  //   - Max Angular Speed: 2π rad/s
  //   - Wheel Diameter: 0.0762m (3 inches)
  //   - Track Width: 26.5 inches
  //   - Wheel Base: 26.5 inches
  //
  // VISION:
  //   - Camera 1: "Arducam_OV9281_USB_Camera" (front)
  //   - Camera 2: "Arducam_OV9281_USB_Camera_2" (back)
  //   - Max Pitch/Roll: 3.0 degrees
  //   - Max Pose Difference: 2.0 meters
  //   - Max Ambiguity: 0.2
  //
  // === CONTROLLER MAPPINGS ===
  //
  // DRIVER (Xbox Controller - Port 0):
  //   - Left Stick: Drive (X/Y translation)
  //   - Right Stick: Rotate (when hub tracking disabled)
  //   - Right Bumper: Enable hub auto-aim
  //   - Left Bumper: Disable hub auto-aim
  //   - Start: Zero gyro heading
  //   - R1: X-stance (brake)
  //
  // OPERATOR (F310 Gamepad - Port 1):
  //   - A Button: Shoot (mode-dependent: manual preset or auto-aim)
  //   - B Button: Eject all mechanisms
  //   - X Button: Deploy intake lift
  //   - Y Button: Retract intake lift
  //   - Left Bumper: Manual hopper feed
  //   - Right Bumper: Spin shooter only (no hopper)
  //   - POV-Up: Toggle ShootOnMove ↔ AutoShoot (automatic mode only)
  //   - POV-Right: Close preset (3000 RPM)
  //   - POV-Down: At Distance preset (5200 RPM)
  //   - POV-Left: Toggle MANUAL ↔ AUTOMATIC mode
  //
  // === TELEMETRY TOPICS (ElasticTelemetry) ===
  //
  // Shooter:
  //   - Shooter/Actual RPM
  //   - Shooter/Target RPM
  //   - Shooter/Target RPM Setpoint
  //   - Shooter/Distance To Hub (m)
  //   - Shooter/Suggested RPM
  //   - Shooter/Angle Error To Hub (deg)
  //   - Shooter/ActivePreset
  //   - Shooter/Mode (MANUAL/AUTOMATIC)
  //   - Shooter/AutomaticMode (boolean)
  //   - Shooter/UseShootOnMove (boolean)
  //   - Shooter/AutoAimMode (SHOOT ON MOVE/AUTO SHOOT)
  //
  // Hopper:
  //   - Hopper/Actual RPM
  //   - Hopper/Target RPM
  //
  // Drive:
  //   - Drive/HubTrackingEnabled (boolean)
  //   - Drive/TotalDriveCurrent
  //   - Drive/FrontLeft/...
  //   - Drive/FrontRight/...
  //   - Drive/RearLeft/...
  //   - Drive/RearRight/...
  //
  // Gyro:
  //   - Gyro/Yaw
  //   - Gyro/Pitch
  //   - Gyro/Roll
  //
  // Vision (per camera):
  //   - Vision/{CameraName}/Status
  //   - Vision/{CameraName}/LatencyMs
  //   - Vision/{CameraName}/TargetCount
  //   - Vision/{CameraName}/Ambiguity
  //   - Vision/{CameraName}/Residual/TranslationMeters
  //   - Vision/{CameraName}/Residual/HeadingDegrees
  //   - Vision/{CameraName}/Residual/PoseAgeMs
  //
  // Game Info:
  //   - Game/Phase (Disabled/Autonomous/Teleop/Test)
  //   - Game/My Team (Red/Blue/Unknown)
  //   - Game/Active HUB (Red Speaker/Blue Speaker/Unknown)
  //   - Game/Game Time (s)
  //   - Game/OurScore
  //   - Game/OpponentScore
  //   - Game/Auto Winner
  //   - Game/CanScoreNow (boolean)
  //   - Game/ScoringWindow (phase description)
  //
  // === COMMON DEBUGGING SCENARIOS ===
  //
  // Motor not spinning:
  //   1. Check CAN ID matches constant definition
  //   2. Verify motor controller shows up in REV Hardware Client
  //   3. Check motor inversion setting (see subsystem constants)
  //   4. Verify current limit isn't being hit (check telemetry)
  //   5. Check if motor is in brake/coast mode as expected
  //
  // Encoder reading wrong values:
  //   1. Check velocity conversion factor (Shooter: 0.5, Hopper: 1.0, Intake: 1.0)
  //   2. Verify gear ratio is correct (Shooter: 2:1)
  //   3. Check if encoder is properly connected to motor controller
  //   4. Verify encoder type matches configuration (relative vs absolute)
  //
  // Swerve module not turning correctly:
  //   1. Check angular offset in DriveConstants (all currently 0)
  //   2. Verify absolute encoder is reading correctly
  //   3. Check turning motor inversion (currently true)
  //   4. Verify PID gains in Configs.java (Kp=1.0)
  //
  // Vision not working:
  //   1. Verify cameras show up in PhotonVision dashboard
  //   2. Check camera names match VisionConstants
  //   3. Verify AprilTag field layout is correct (2026 Rebuilt Welded)
  //   4. Check validation thresholds (pitch/roll, pose difference, ambiguity)
  //   5. Monitor Vision/{CameraName}/Status telemetry
  //
  // Auto-aim not working:
  //   1. Check hub tracking is enabled (Drive/HubTrackingEnabled)
  //   2. Verify hub positions in FieldConstants
  //   3. Check PID controller gains (turnToPointController: Kp=4.0)
  //   4. Verify tolerance (2 degrees)
  //   5. Check alliance color is detected correctly
  //
  // Shooter RPM incorrect:
  //   1. Verify gear ratio (2:1) and conversion factor (0.5)
  //   2. Check PID gains (Kp=0.0001, Kv=0.0021)
  //   3. Monitor Shooter/Actual RPM vs Shooter/Target RPM
  //   4. Verify voltage compensation is enabled (12.0V)
  //   5. Check current limit (60A)
  //
}
