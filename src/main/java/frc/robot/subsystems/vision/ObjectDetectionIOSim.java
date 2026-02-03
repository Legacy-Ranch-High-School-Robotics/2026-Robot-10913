package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;

public class ObjectDetectionIOSim implements ObjectDetectionIO {
  private final Transform3d robotToCamera;
  private Pose2d robotPose = new Pose2d(); // Track robot pose to calculate relative positions

  public ObjectDetectionIOSim(Transform3d robotToCamera) {
    this.robotToCamera = robotToCamera;
  }

  // Update robot pose to calculate relative game piece positions
  public void updateRobotPose(Pose2d pose) {
    this.robotPose = pose;
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    // Get all game pieces from the arena
    // Note: Assuming "Coral" and "Algae" are the game piece types we care about
    var pieces = new ArrayList<DetectedObject>();

    // Check Coral
    var corals = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
    if (corals != null) {
      for (var pose : corals) {
        checkAndAddDetection(pose, "Coral", pieces);
      }
    }

    // Check Algae
    var algae = SimulatedArena.getInstance().getGamePiecesArrayByType("Algae");
    if (algae != null) {
      for (var pose : algae) {
        checkAndAddDetection(pose, "Algae", pieces);
      }
    }

    // Populate inputs
    inputs.timestamps =
        new double[pieces.size()]; // Determining timestamp is tricky here, stick to loop time
    inputs.classNames = new String[pieces.size()];
    inputs.confidences = new double[pieces.size()];
    inputs.distances = new double[pieces.size()];
    inputs.yawAngles = new double[pieces.size()];

    for (int i = 0; i < pieces.size(); i++) {
      var obj = pieces.get(i);
      inputs.timestamps[i] = 0.0; // Current loop
      inputs.classNames[i] = obj.className;
      inputs.confidences[i] = obj.confidence;
      inputs.distances[i] = obj.distance;
      inputs.yawAngles[i] = obj.yaw.getRadians();
    }
  }

  private void checkAndAddDetection(Pose3d piecePose, String className, List<DetectedObject> list) {
    // 1. Calculate relative position to ROBOT first (ignoring camera height/pitch for simple
    // horizontal FOV check)
    Translation2d pieceTrans = piecePose.toPose2d().getTranslation();
    Translation2d robotTrans = robotPose.getTranslation();
    Rotation2d robotRot = robotPose.getRotation();

    // Transform world game piece to robot-relative
    // (This math simplifies to 2D for detecting ground objects)
    Translation2d relativeToRobot = pieceTrans.minus(robotTrans).rotateBy(robotRot.unaryMinus());

    // Transform robot-relative to camera-relative (using the camera's translation/yaw)
    // Camera Transform: X forward, Y left.
    // Ignoring Z and Pitch for simple "is in front" check.
    Translation2d camearOffset = robotToCamera.getTranslation().toTranslation2d();
    Rotation2d cameraYaw = robotToCamera.getRotation().toRotation2d();

    Translation2d relativeToCamera =
        relativeToRobot.minus(camearOffset).rotateBy(cameraYaw.unaryMinus());

    // 2. Check FOV
    // Assume ~60 degree horizontal FOV (+/- 30 degrees)
    // Assume max distance 5 meters
    double distance = relativeToCamera.getNorm();
    Rotation2d angle = relativeToCamera.getAngle();

    if (distance > 0.1 && distance < 5.0 && Math.abs(angle.getDegrees()) < 30.0) {
      // Visible!
      // Add some noise simulation if desired
      list.add(new DetectedObject(className, 0.9, distance, angle));
    }
  }
}
