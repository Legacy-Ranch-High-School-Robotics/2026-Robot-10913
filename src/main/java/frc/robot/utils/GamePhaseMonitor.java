package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamePhaseMonitor extends SubsystemBase {
  private final SendableChooser<String> autoWinnerChooser = new SendableChooser<>();
  private final Timer matchTimer = new Timer();
  private boolean wasAuto = false;
  private boolean timerStarted = false;

  public GamePhaseMonitor() {
    autoWinnerChooser.setDefaultOption("Auto (From FMS)", "Auto");
    autoWinnerChooser.addOption("Red Won Auto (Override)", "Red");
    autoWinnerChooser.addOption("Blue Won Auto (Override)", "Blue");
    SmartDashboard.putData("Game/Auto Winner", autoWinnerChooser);
  }

  @Override
  public void periodic() {
    boolean isAutoNow = DriverStation.isAutonomousEnabled();

    // Start or reset timer strictly at the beginning of Autonomous
    if (isAutoNow && !wasAuto) {
      matchTimer.restart();
      timerStarted = true;
    }
    wasAuto = isAutoNow;

    double elapsed = matchTimer.get();
    String phase = "UNKNOWN";
    String activeHub = "UNKNOWN";

    // Strictly chronological status based only on rules duration mapping
    if (!timerStarted) {
      phase = "DISABLED (Waiting for Auto)";
      activeHub = "NONE";
    } else {
      if (elapsed < 20.0) {
        phase = "AUTO";
        activeHub = "BOTH";
      } else if (elapsed < 30.0) {
        phase = "TELEOP TRANSITION SHIFT";
        activeHub = "BOTH";
      } else if (elapsed < 55.0) {
        phase = "SHIFT 1";
        activeHub = getHubForShift(1);
      } else if (elapsed < 80.0) {
        phase = "SHIFT 2";
        activeHub = getHubForShift(2);
      } else if (elapsed < 105.0) {
        phase = "SHIFT 3";
        activeHub = getHubForShift(3);
      } else if (elapsed < 130.0) {
        phase = "SHIFT 4";
        activeHub = getHubForShift(4);
      } else if (elapsed < 160.0) {
        phase = "END GAME";
        activeHub = "BOTH";
      } else {
        phase = "MATCH OVER";
        activeHub = "NONE";
      }
    }

    String hubStatus = "UNKNOWN";
    String ourAllianceStr = "UNKNOWN";

    var allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isPresent()) {
      ourAllianceStr = allianceOpt.get() == DriverStation.Alliance.Red ? "RED" : "BLUE";
      if (activeHub.equals("BOTH")) {
        hubStatus = "ACTIVE";
      } else if (activeHub.equals("NONE") || activeHub.startsWith("UNKNOWN")) {
        hubStatus = "UNKNOWN";
      } else if (activeHub.equals(ourAllianceStr)) {
        hubStatus = "ACTIVE";
      } else {
        hubStatus = "INACTIVE";
      }
    }

    // Publish directly to the "Game/" folder to avoid Subsystem widget rendering issues
    SmartDashboard.putNumber(
        "Game/Game Time (s)", timerStarted ? Math.max(0, Math.round(elapsed)) : 0);
    SmartDashboard.putString("Game/Phase", phase);
    SmartDashboard.putString("Game/My Team", ourAllianceStr);
    SmartDashboard.putString("Game/Active HUB", hubStatus);
  }

  private String getHubForShift(int shiftNumber) {
    String autoWinner = autoWinnerChooser.getSelected();
    if (autoWinner == null || autoWinner.equals("Auto")) {
      String fmsMessage = DriverStation.getGameSpecificMessage();
      if (fmsMessage != null) {
        if (fmsMessage.equals("R")) {
          autoWinner = "Red";
        } else if (fmsMessage.equals("B")) {
          autoWinner = "Blue";
        } else {
          autoWinner = "Unknown";
        }
      } else {
        autoWinner = "Unknown";
      }
    }

    boolean redWon = autoWinner.equals("Red");
    boolean blueWon = autoWinner.equals("Blue");

    if (redWon) {
      return (shiftNumber % 2 != 0) ? "BLUE" : "RED";
    } else if (blueWon) {
      return (shiftNumber % 2 != 0) ? "RED" : "BLUE";
    } else {
      return "UNKNOWN - Waiting for FMS or Override";
    }
  }
}
