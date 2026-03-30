package frc.robot.sim;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.hopper.Hopper;

/**
 * Simulates the hopper feed wheel using WPILib's FlywheelSim and REVLib's SparkMaxSim.
 *
 * <p>Simple 1:1 flywheel simulation — the feed wheel has low inertia so it responds quickly.
 */
public class HopperSim {
  private final Hopper m_hopper;
  private final FlywheelSim m_flywheelSim;
  private final SparkMaxSim m_motorSim;

  public HopperSim(Hopper hopper) {
    m_hopper = hopper;
    DCMotor motor = DCMotor.getNEO(1);
    m_flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(motor, SimConstants.kHopperMOI, 1.0), motor);
    m_motorSim = new SparkMaxSim(hopper.getHopperMotor(), motor);
  }

  /** Call every 20 ms from simulationPeriodic. */
  public void update() {
    double voltage = m_hopper.getHopperMotor().getAppliedOutput() * 12.0;

    m_flywheelSim.setInputVoltage(voltage);
    m_flywheelSim.update(0.02);

    // 1:1 gearing — motor RPM equals wheel RPM
    double motorRPM = m_flywheelSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);

    m_motorSim.iterate(motorRPM, 12.0, 0.02);
  }
}
