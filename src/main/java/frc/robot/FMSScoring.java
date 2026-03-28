package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.telemetry.ElasticTelemetry;

public class FMSScoring {
  private final NetworkTable fmsTable;
  private boolean isFMSConnected = false;

  public FMSScoring() {
    fmsTable = NetworkTableInstance.getDefault().getTable("FMSInfo");
  }

  public void periodic() {
    isFMSConnected = DriverStation.isFMSAttached();
    
    if (!isFMSConnected) {
      ElasticTelemetry.setString("Game/Auto Winner", "N/A - No FMS");
      ElasticTelemetry.setBoolean("Game/CanScoreNow", false);
      return;
    }

    updateScores();
    updateScoringWindow();
  }

  private void updateScores() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return;
    }

    boolean isRed = alliance.get() == DriverStation.Alliance.Red;
    
    int ourScore = (int) fmsTable.getEntry(isRed ? "RedScore" : "BlueScore").getDouble(0);
    int opponentScore = (int) fmsTable.getEntry(isRed ? "BlueScore" : "RedScore").getDouble(0);

    ElasticTelemetry.setNumber("Game/OurScore", ourScore);
    ElasticTelemetry.setNumber("Game/OpponentScore", opponentScore);

    String winner;
    if (ourScore > opponentScore) {
      winner = "Us (" + ourScore + " pts)";
    } else if (opponentScore > ourScore) {
      winner = "Opponent (" + opponentScore + " pts)";
    } else if (ourScore == 0 && opponentScore == 0) {
      winner = "N/A";
    } else {
      winner = "Tie (" + ourScore + " pts)";
    }
    
    ElasticTelemetry.setString("Game/Auto Winner", winner);
  }

  private void updateScoringWindow() {
    boolean canScore = false;
    String reason = "";

    if (DriverStation.isDisabled()) {
      reason = "Robot Disabled";
    } else if (DriverStation.isAutonomous()) {
      canScore = true;
      reason = "AUTO - ACTIVE hubs score";
    } else if (DriverStation.isTeleop()) {
      double matchTime = DriverStation.getMatchTime();
      
      if (matchTime > 120) {
        canScore = true;
        reason = "TELEOP - Alliance Shift 1 (all hubs active)";
      } else if (matchTime > 90) {
        canScore = true;
        reason = "TELEOP - Alliance Shift 2 (ACTIVE hubs only)";
      } else if (matchTime > 60) {
        canScore = true;
        reason = "TELEOP - Alliance Shift 3 (ACTIVE hubs only)";
      } else if (matchTime > 30) {
        canScore = true;
        reason = "TELEOP - Alliance Shift 4 (ACTIVE hubs only)";
      } else {
        canScore = true;
        reason = "TELEOP - End Game (ACTIVE hubs only)";
      }
    } else {
      reason = "Unknown Phase";
    }

    ElasticTelemetry.setBoolean("Game/CanScoreNow", canScore);
    ElasticTelemetry.setString("Game/ScoringWindow", reason);
  }

  public boolean isFMSConnected() {
    return isFMSConnected;
  }

  public boolean canScoreNow() {
    if (!isFMSConnected) {
      return false;
    }
    return DriverStation.isAutonomous() || DriverStation.isTeleop();
  }
}
