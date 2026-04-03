# FRC Robot Programming Guide - Team 10913

## Table of Contents
1. [Introduction](#introduction)
2. [Development Environment Setup](#development-environment-setup)
3. [Project Structure](#project-structure)
4. [Core Concepts](#core-concepts)
5. [Subsystems](#subsystems)
6. [Commands](#commands)
7. [Controller Bindings](#controller-bindings)
8. [Autonomous Programming](#autonomous-programming)
9. [Testing and Debugging](#testing-and-debugging)
10. [Best Practices](#best-practices)

---

## Introduction

This guide explains how to program the 2026 FRC robot for Team 10913. The robot uses the **Command-Based Programming** paradigm with WPILib and is written in Java.

### Robot Capabilities
- **Swerve Drive**: Holonomic movement with MAXSwerve modules
- **Shooter**: Dual-motor flywheel system with velocity control
- **Hopper**: Single-motor feeder with PID velocity control
- **Intake**: Dual-motor system (roller + lift mechanism)
- **Vision**: AprilTag detection for auto-aiming

---

## Development Environment Setup

### Required Software
1. **WPILib VS Code** (includes Java JDK, Gradle, and FRC tools)
2. **Git** for version control
3. **FRC Driver Station** for robot control
4. **PathPlanner** for autonomous path creation

### Installation Steps
1. Download WPILib installer from [wpilib.org](https://wpilib.org)
2. Run installer and select all components
3. Clone this repository:
   ```bash
   git clone <repository-url>
   cd 2026-Robot-10913-15
   ```
4. Open project in WPILib VS Code

### Building the Project
```bash
# Build code
.\gradlew build

# Deploy to robot (must be connected)
.\gradlew deploy

# Run simulator
.\gradlew simulateJava
```

---

## Project Structure

```
src/main/java/frc/robot/
├── Robot.java              # Main robot class
├── RobotContainer.java     # Subsystem/command initialization
├── Constants.java          # Global constants
├── commands/               # Command classes
│   ├── intake/            # Intake commands
│   ├── shooter/           # Shooter commands
│   └── ...
├── subsystems/            # Subsystem classes
│   ├── DriveSubsystem.java
│   ├── intake/
│   │   ├── Intake.java
│   │   └── IntakeConstants.java
│   ├── hopper/
│   │   ├── Hopper.java
│   │   └── HopperConstants.java
│   ├── shooter/
│   │   ├── Shooter.java
│   │   └── ShooterConstants.java
│   └── vision/
└── telemetry/             # Dashboard/logging utilities
```

---

## Core Concepts

### Command-Based Programming
The robot code follows these principles:

1. **Subsystems**: Represent physical mechanisms (drive, shooter, intake, etc.)
2. **Commands**: Define actions that subsystems perform
3. **Triggers/Buttons**: Bind commands to controller inputs
4. **Scheduler**: Runs commands and manages subsystem states

### Subsystem Lifecycle
```java
public class ExampleSubsystem extends SubsystemBase {
    // Constructor: Initialize hardware
    public ExampleSubsystem() {
        motor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
    }
    
    // Periodic: Runs every 20ms
    @Override
    public void periodic() {
        // Update telemetry, check sensors, etc.
    }
    
    // Methods: Control the subsystem
    public void doSomething() {
        motor.setVoltage(12.0);
    }
}
```

### Command Types

**InstantCommand**: Executes once immediately
```java
new InstantCommand(() -> subsystem.doSomething(), subsystem)
```

**RunCommand**: Runs continuously while active
```java
new RunCommand(() -> subsystem.continuousAction(), subsystem)
```

**Custom Commands**: Extend `Command` class for complex logic
```java
public class CustomCommand extends Command {
    public CustomCommand(Subsystem subsystem) {
        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() { /* Setup */ }
    
    @Override
    public void execute() { /* Run repeatedly */ }
    
    @Override
    public void end(boolean interrupted) { /* Cleanup */ }
    
    @Override
    public boolean isFinished() { return false; }
}
```

---

## Subsystems

### Drive Subsystem
**Location**: `src/main/java/frc/robot/subsystems/DriveSubsystem.java`

**Purpose**: Controls swerve drive modules for holonomic movement

**Key Methods**:
- `drive(xSpeed, ySpeed, rot, fieldRelative)`: Drive the robot
- `setX()`: Lock wheels in X-formation
- `zeroHeading()`: Reset gyro to 0°
- `getPose()`: Get robot position on field
- `setTrackingHub(boolean)`: Enable/disable auto-aiming

**Hardware**:
- 4x MAXSwerve modules (CAN IDs 1-8)
- Pigeon 2 IMU (CAN ID 9)

### Shooter Subsystem
**Location**: `src/main/java/frc/robot/subsystems/shooter/Shooter.java`

**Purpose**: Launches game pieces using dual flywheels

**Key Methods**:
- `setVelocity(rpm)`: Set target flywheel speed
- `atTargetVelocity()`: Check if ready to shoot
- `stop()`: Stop flywheels
- `eject()`: Reverse to clear jams

**Hardware**:
- 2x SparkMax NEO motors (CAN IDs 13, 14)
- Closed-loop velocity control with PID + feedforward

**Constants** (`ShooterConstants.java`):
```java
public static final double shooterRPM = 5000.0;      // Default speed
public static final double closePresetRPM = 3500.0;  // Close shot
public static final double shooterKp = 0.0001;       // PID P gain
public static final double shooterKv = 0.0021;       // Feedforward
```

### Hopper Subsystem
**Location**: `src/main/java/frc/robot/subsystems/hopper/Hopper.java`

**Purpose**: Feeds game pieces from intake to shooter

**Key Methods**:
- `setVelocity(rpm)`: Set feed speed
- `stop()`: Stop feeding
- `eject()`: Reverse to clear jams
- `atTargetVelocity()`: Check if at speed

**Hardware**:
- 1x SparkMax NEO motor (CAN ID 12)
- Velocity control with PID

**Operation Modes**:
- **Idle**: 1500 RPM (hold game piece)
- **Feed**: 5000 RPM (launch into shooter)
- **Eject**: -6V (reverse)

### Intake Subsystem
**Location**: `src/main/java/frc/robot/subsystems/intake/Intake.java`

**Purpose**: Collects game pieces from floor

**Key Methods**:
- `intake()`: Run roller to collect
- `outtake()`: Reverse roller
- `feed()`: Feed to hopper
- `liftDeploy()`: Lower intake to floor
- `liftRetract()`: Raise intake
- `isLiftDeployed()`: Check if fully deployed

**Hardware**:
- Roller motor: SparkMax NEO (CAN ID 11)
- Lift motor: SparkMax NEO (CAN ID 10)

**Position Control**:
```java
public static final double deployedPosition = 3.476;   // Encoder rotations
public static final double retractedPosition = 0.0;
public static final double liftPositionTolerance = 0.5;
```

---

## Commands

### Creating a New Command

1. **Create command file** in appropriate folder:
```java
package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends Command {
    private final Intake m_intake;
    
    public IntakeCommand(Intake intake) {
        m_intake = intake;
        addRequirements(intake);  // Prevents conflicts
    }
    
    @Override
    public void initialize() {
        // Deploy intake when command starts
        m_intake.liftDeploy();
    }
    
    @Override
    public void execute() {
        // Run intake roller
        m_intake.intake();
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop when command ends
        m_intake.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;  // Runs until interrupted
    }
}
```

2. **Register in RobotContainer**:
```java
private void configureButtonBindings() {
    new JoystickButton(controller, Button.kA.value)
        .whileTrue(new IntakeCommand(m_intake));
}
```

### Command Composition

**Sequential Commands**:
```java
Commands.sequence(
    new DeployIntake(m_intake),
    new IntakeCommand(m_intake),
    new RetractIntake(m_intake)
)
```

**Parallel Commands**:
```java
Commands.parallel(
    new SpinUpShooter(m_shooter),
    new AimAtTarget(m_drive)
)
```

**Conditional Commands**:
```java
Commands.either(
    new ShootHigh(m_shooter),
    new ShootLow(m_shooter),
    () -> distance > 3.0  // Condition
)
```

---

## Controller Bindings

### Driver Controller (Xbox)
| Button | Action |
|--------|--------|
| Left Stick | Translate (X/Y movement) |
| Right Stick X | Rotate |
| Right Bumper | Enable hub tracking |
| Left Bumper | Disable hub tracking |
| Start | Zero heading |

### Operator Controller (F310/Xbox)
| Button | Action |
|--------|--------|
| A | Launch (shooter + hopper) |
| B | Eject all mechanisms |
| X | Deploy intake |
| Y | Retract intake |
| Left Bumper | Outtake |
| Right Bumper | Spin up shooter only |
| POV Up | Toggle shoot-on-move |
| POV Left | Toggle auto/manual mode |
| POV Right | Close shot preset |
| POV Down | Distance shot preset |

### Adding New Bindings

In `RobotContainer.java`:
```java
private void configureButtonBindings() {
    // Button binding example
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileTrue(launchCommand)      // Hold to run
        .onFalse(stopLaunchCommand);   // Release to stop
    
    // POV (D-pad) binding
    new POVButton(m_operatorController, 0)  // 0=Up, 90=Right, 180=Down, 270=Left
        .onTrue(new InstantCommand(() -> doSomething()));
    
    // Trigger binding
    new Trigger(m_shooter::atTargetVelocity)
        .onTrue(new InstantCommand(() -> rumbleController()));
}
```

---

## Autonomous Programming

### Using PathPlanner

1. **Create paths** in PathPlanner GUI
2. **Register named commands** in `RobotContainer`:
```java
private void configurePathPlannerCommands() {
    NamedCommands.registerCommand("SpinUpShooter", new SpinUpShooter(m_shooter));
    NamedCommands.registerCommand("Shoot", new ShootCommand(m_shooter, m_hopper));
    NamedCommands.registerCommand("Intake", new IntakeCommand(m_intake));
}
```

3. **Add event markers** in PathPlanner at desired positions
4. **Select auto** from dashboard chooser

### Auto Builder Configuration
The robot uses `AutoBuilder` for PathPlanner integration:
```java
AutoBuilder.configure(
    m_robotDrive::getPose,           // Pose supplier
    m_robotDrive::resetOdometry,     // Reset pose
    m_robotDrive::getChassisSpeeds,  // Current speeds
    m_robotDrive::driveRobotRelative,// Drive command
    new PPHolonomicDriveController(...),
    config,
    allianceSupplier,
    m_robotDrive
);
```

### Creating Custom Autos
```java
public Command getCustomAuto() {
    return Commands.sequence(
        new InstantCommand(() -> m_robotDrive.resetOdometry(startPose)),
        new DeployIntake(m_intake),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("MyPath")),
        new ShootCommand(m_shooter, m_hopper)
    );
}
```

---

## Testing and Debugging

### Telemetry
Use `ElasticTelemetry` for dashboard logging:
```java
// In subsystem periodic()
ElasticTelemetry.setNumber("Shooter/Actual RPM", encoder.getVelocity());
ElasticTelemetry.setBoolean("Intake/IsDeployed", isLiftDeployed());
ElasticTelemetry.setString("Auto/CurrentPath", pathName);
```

### Simulation
```bash
.\gradlew simulateJava
```
- Test code without robot hardware
- Verify logic and command sequences
- Use Glass for visualization

### Common Debugging Steps
1. **Check CAN IDs**: Verify in Phoenix Tuner or REV Hardware Client
2. **Motor direction**: Set `inverted` in constants if backwards
3. **Current limits**: Prevent brownouts with `smartCurrentLimit()`
4. **Encoder values**: Log to dashboard to verify sensor readings
5. **Command conflicts**: Ensure `addRequirements()` is called

### Driver Station Logs
- Located in: `C:\Users\Public\Documents\FRC\Log Files`
- Use DriverStation Log Viewer to analyze

---

## Best Practices

### Code Organization
- **One subsystem per mechanism**: Don't combine unrelated hardware
- **Constants in separate files**: Use `SubsystemConstants.java` pattern
- **Static imports for constants**: `import static frc.robot.subsystems.intake.IntakeConstants.*;`

### Safety
- **Current limits**: Always set on motor controllers
- **Voltage compensation**: Use `voltageCompensation(12.0)` for consistency
- **Soft limits**: Prevent mechanism damage with position limits
- **Emergency stops**: Implement stop methods for all subsystems

### Performance
- **Avoid blocking code**: Never use `Thread.sleep()` in robot code
- **Minimize periodic() work**: Keep it fast (<20ms)
- **Use suppliers**: Pass `() -> value` instead of updating variables
- **Cache hardware objects**: Don't recreate motor controllers

### Version Control
- **Commit often**: Small, focused commits
- **Descriptive messages**: "Add intake deploy command" not "fix stuff"
- **Test before pushing**: Ensure code builds and deploys
- **Branch for features**: Create branches for major changes

### Motor Controller Configuration
```java
var config = new SparkMaxConfig();
config
    .inverted(motorInverted)
    .idleMode(IdleMode.kBrake)           // or kCoast
    .smartCurrentLimit(currentLimit)
    .voltageCompensation(12.0);

motor.configure(
    config,
    ResetMode.kResetSafeParameters,      // Reset to defaults first
    PersistMode.kPersistParameters       // Save to flash
);
```

### PID Tuning Process
1. **Set Kp, Ki, Kd to 0**
2. **Increase Kp** until oscillation occurs
3. **Reduce Kp** by 50%
4. **Add Kd** to reduce overshoot
5. **Add Ki** only if steady-state error exists
6. **Add feedforward (Kv)** for velocity control

### Troubleshooting Checklist
- [ ] Robot code deployed successfully?
- [ ] Driver Station shows "Robot Code" green?
- [ ] CAN devices show up in Phoenix Tuner/REV Client?
- [ ] Motors spin in correct direction?
- [ ] Sensors reading reasonable values?
- [ ] No brownouts (check voltage on Driver Station)?
- [ ] Joystick/controller connected?
- [ ] Correct team number in `.wpilib/wpilib_preferences.json`?

---

## Additional Resources

- **WPILib Documentation**: https://docs.wpilib.org
- **REV Robotics Docs**: https://docs.revrobotics.com
- **PathPlanner Docs**: https://pathplanner.dev
- **Chief Delphi Forums**: https://www.chiefdelphi.com
- **FRC Discord**: https://discord.gg/frc

---

## Quick Reference

### Common Imports
```java
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
```

### Team Information
- **Team Number**: 10913
- **Robot Name**: 2026 Robot
- **Programming Language**: Java
- **Framework**: WPILib Command-Based

---

*Last Updated: 2026 Season*
*Maintained by Team 10913 Programming Team*
