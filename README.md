

FRC Robot Code for Team 10913 - 2026 Season

## Project Structure

This project combines:
- **Swerve Drive** from Working Swerve code (no AdvantageKit)
- **Shooter Subsystem** from special program (AdvantageKit removed)
- **Intake Subsystem** from special program (AdvantageKit removed)

## Subsystems

### Drive Subsystem
- MAXSwerve modules with NEO motors
- Pigeon 2 IMU for heading
- Field-relative and robot-relative driving modes

### Shooter Subsystem
- Dual SparkMax motors (CAN IDs 12, 13)
- Velocity control with PID
- Preset speeds for speaker and amp scoring

### Intake Subsystem
- Single SparkMax motor (CAN ID 10)
- Beam break sensor on DIO 0
- Intake, outtake, and feed modes

## Controller Mapping

### Drive Controls
- **Left Stick**: Translation (X/Y movement)
- **Right Stick X**: Rotation
- **Start Button**: Zero heading
- **R1**: X-formation (lock wheels)

### Shooter Controls
- **A Button**: Shoot at speaker speed (5000 RPM)
- **B Button**: Shoot at amp speed (2500 RPM)

### Intake Controls
- **X Button**: Intake game piece
- **Y Button**: Outtake game piece
- **Right Bumper**: Feed to shooter

## Building and Deploying

```bash
# Build the project
.\gradlew build

# Deploy to robot
.\gradlew deploy

# Simulate
.\gradlew simulateJava
```

## Configuration

Update CAN IDs and other constants in:
- `src/main/java/frc/robot/Constants.java` - Drive constants
- `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java` - Shooter constants
- `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java` - Intake constants

## Team Number

Team: **10913**

Update team number in `.wpilib/wpilib_preferences.json` if needed.
