package frc.robot.sim;

import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Simulates the shooter flywheel using WPILib's FlywheelSim and REVLib's SparkFlexSim.
 *
 * <p>The flywheel has real inertia, so it takes time to spin up to the target RPM — just like the
 * real robot. The PID + feedforward in the SparkFlex runs internally via SparkFlexSim.iterate().
 */
public class ShooterSim {
  private final Shooter m_shooter;
  private final FlywheelSim m_flywheelSim;
  private final SparkFlexSim m_motorSim;

  public ShooterSim(Shooter shooter) {
    m_shooter = shooter;
    DCMotor motor = DCMotor.getNeoVortex(1);
    m_flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                motor, SimConstants.kShooterFlywheelMOI, SimConstants.kShooterGearing),
            motor);
    m_motorSim = new SparkFlexSim(shooter.getTopMotor(), motor);
  }

  /** Call every 20 ms from simulationPeriodic. */
  public void update() {
    // 1. Read voltage the motor controller is applying
    double voltage = m_shooter.getTopMotor().getAppliedOutput() * 12.0;

    // 2. Step the flywheel physics
    m_flywheelSim.setInputVoltage(voltage);
    m_flywheelSim.update(0.02);

    // 3. Convert flywheel (output) velocity to motor shaft RPM
    double motorRPM =
        m_flywheelSim.getAngularVelocityRadPerSec()
            * SimConstants.kShooterGearing
            * 60.0
            / (2.0 * Math.PI);

    // 4. Feed back into SparkFlexSim (updates encoder + runs PID for next cycle)
    m_motorSim.iterate(motorRPM, 12.0, 0.02);
  }
}
