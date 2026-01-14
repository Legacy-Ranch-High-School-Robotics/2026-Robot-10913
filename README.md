# MAXSwerve Java Template v2026.0

See [the online changelog](https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/CHANGELOG.md) for information about updates to the template that may have been released since you created your project.

## Description

A template project for an FRC swerve drivetrain that uses REV MAXSwerve Modules.

Note that this template is designed for a drivetrain composed of four MAXSwerve Modules, each configured with two SPARKS MAX, a NEO as the driving motor, a NEO 550 as the turning motor, and a REV Through Bore Encoder as the absolute turning encoder. If you are using SPARK Flex for either the drive motor or turning motor, you will need to update the classes accordingly.

To get started, make sure you have calibrated the zero offsets for the absolute encoders in Hardware Client 2 using the `Absolute Encoder` utility under the associated turning SPARK devices.

## Prerequisites

* SPARK MAX Firmware v26.1.0
* REVLib v2026.0.0

## Configuration

It is possible that this project will not work for your robot right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!

These values can be adjusted in the `Configs.java` and `Constants.java` files.

## Simulation

To run this project in simulation:

1. Open the Command Palette in VS Code (`Ctrl+Shift+P` or `F1`).
2. Type and select **"WPILib: Simulate Robot Code"**.
3. When prompted, ensure **"HALSim Gui"** is selected.
4. The Simulation GUI will launch, allowing you to test robot logic and autonomous routines.
5. To view the Field widget, navigate to **NetworkTables** -> **SmartDashboard** -> **Field** in the menu bar.
