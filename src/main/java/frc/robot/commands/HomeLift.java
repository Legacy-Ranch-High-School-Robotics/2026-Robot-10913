package frc.robot.commands;

import static frc.robot.subsystems.intakelift.IntakeLiftConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakelift.IntakeLift;

public class HomeLift extends Command {
  private final IntakeLift m_lift;

  public HomeLift(IntakeLift lift) {
    m_lift = lift;
    addRequirements(m_lift);
  }

  @Override
  public void initialize() {
    m_lift.setHomingState(IntakeLift.HomingState.FINDING_ZERO);
  }

  @Override
  public void execute() {
    switch (m_lift.getHomingState()) {
      case FINDING_ZERO:
        m_lift.applyVoltage(homingRetractVoltage);
        if (m_lift.getCurrentAmps() > homingCurrentThreshold) {
          m_lift.applyVoltage(0);
          m_lift.zeroEncoder();
          m_lift.setHomingState(IntakeLift.HomingState.FINDING_MAX);
        }
        break;
      case FINDING_MAX:
        m_lift.applyVoltage(homingDeployVoltage);
        if (m_lift.getCurrentAmps() > homingCurrentThreshold) {
          m_lift.applyVoltage(0);
          m_lift.saveMaxPosition();
          m_lift.setHomingState(IntakeLift.HomingState.HOMED);
        }
        break;
      case HOMED:
        m_lift.liftStop();
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return m_lift.getHomingState() == IntakeLift.HomingState.HOMED;
  }

  @Override
  public void end(boolean interrupted) {
    m_lift.liftStop();
  }
}
