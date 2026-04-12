package frc.robot.sim;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.Intake;

/**
 * Simulates both intake motors:
 *
 * <ul>
 *   <li><b>Roller</b>: FlywheelSim (NEO 550, 1:1) — spins the intake rollers
 *   <li><b>Lift (arm)</b>: SingleJointedArmSim (NEO 550, belt-driven ~13.9:1) — sweeps 0°
 *       (horizontal/deployed) to 90° (vertical/retracted) with hard stops at both ends
 * </ul>
 */
public class IntakeSim {
  private final Intake m_intake;

  // Roller simulation
  private final FlywheelSim m_rollerSim;
  private final SparkMaxSim m_rollerMotorSim;

  // Lift (arm) simulation
  private final SingleJointedArmSim m_armSim;
  private final SparkMaxSim m_liftMotorSim;

  public IntakeSim(Intake intake) {
    m_intake = intake;

    // Roller: NEO 550, 1:1
    DCMotor rollerMotor = DCMotor.getNeo550(1);
    m_rollerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(rollerMotor, SimConstants.kIntakeRollerMOI, 1.0),
            rollerMotor);
    m_rollerMotorSim = new SparkMaxSim(intake.getMotor(), rollerMotor);

    // Lift arm: NEO 550, belt-driven ~13.9:1
    DCMotor liftMotor = DCMotor.getNeo550(1);
    m_armSim =
        new SingleJointedArmSim(
            liftMotor,
            SimConstants.kIntakeArmGearing,
            SingleJointedArmSim.estimateMOI(
                SimConstants.kIntakeArmLengthMeters, SimConstants.kIntakeArmMassKg),
            SimConstants.kIntakeArmLengthMeters,
            SimConstants.kIntakeArmMinAngleRad,
            SimConstants.kIntakeArmMaxAngleRad,
            true, // simulate gravity
            SimConstants.kIntakeArmStartingAngleRad);
    m_liftMotorSim = new SparkMaxSim(intake.getLiftMotor(), liftMotor);
  }

  /** Call every 20 ms from simulationPeriodic. */
  public void update() {
    updateRoller();
    updateLiftArm();
  }

  private void updateRoller() {
    double voltage = m_intake.getMotor().getAppliedOutput() * 12.0;

    m_rollerSim.setInputVoltage(voltage);
    m_rollerSim.update(0.02);

    double motorRPM = m_rollerSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
    m_rollerMotorSim.iterate(motorRPM, 12.0, 0.02);
  }

  private void updateLiftArm() {
    // Positive motor voltage deploys (lowers) the arm, but in SingleJointedArmSim
    // positive voltage increases angle. Deploy = angle decreases (π/2 → 0), so negate.
    double motorVoltage = m_intake.getLiftMotor().getAppliedOutput() * 12.0;
    m_armSim.setInputVoltage(-motorVoltage);
    m_armSim.update(0.02);

    // Convert arm angular velocity to motor shaft RPM (negated to match motor convention)
    double motorVelRadPerSec = -m_armSim.getVelocityRadPerSec() * SimConstants.kIntakeArmGearing;
    double motorRPM = motorVelRadPerSec * 60.0 / (2.0 * Math.PI);
    m_liftMotorSim.iterate(motorRPM, 12.0, 0.02);
  }
}
