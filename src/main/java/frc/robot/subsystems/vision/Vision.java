package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO frontTagIO;
  private final VisionIOInputsAutoLogged frontTagInputs = new VisionIOInputsAutoLogged();

  private final VisionIO backTagIO;
  private final VisionIOInputsAutoLogged backTagInputs = new VisionIOInputsAutoLogged();

  private final ObjectDetectionIO frontMLIO;
  private final ObjectDetectionIOInputsAutoLogged frontMLInputs =
      new ObjectDetectionIOInputsAutoLogged();

  private final Drive drive;

  public Vision(VisionIO frontTagIO, VisionIO backTagIO, ObjectDetectionIO frontMLIO, Drive drive) {
    this.frontTagIO = frontTagIO;
    this.backTagIO = backTagIO;
    this.frontMLIO = frontMLIO;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    // 1. Update Inputs
    frontTagIO.updateInputs(frontTagInputs);
    backTagIO.updateInputs(backTagInputs);
    frontMLIO.updateInputs(frontMLInputs);

    // 2. Process Logs
    Logger.processInputs("Vision/FrontTag", frontTagInputs);
    Logger.processInputs("Vision/BackTag", backTagInputs);
    Logger.processInputs("Vision/FrontML", frontMLInputs);

    // 3. Update Drive with Vision Measurements
    List<Pose3d> allVisionPoses = new ArrayList<>();
    allVisionPoses.addAll(updateDriveWithVision(frontTagInputs));
    allVisionPoses.addAll(updateDriveWithVision(backTagInputs));

    // Log all valid vision poses for visualization (Ghost Robot)
    Logger.recordOutput("Vision/VisionPoses", allVisionPoses.toArray(new Pose3d[0]));

    // 4. Update Object Detection (Log for now)
    // The "frontMLInputs" are already logged above, so AdvantageScope can see them.
  }

  private List<Pose3d> updateDriveWithVision(VisionIO.VisionIOInputs inputs) {
    List<Pose3d> validPoses = new ArrayList<>();

    for (int i = 0; i < inputs.size(); i++) {
      // Only add good estimates when more than 'minPoses' are seen
      if (inputs.tagCounts[i] >= VisionConstants.minTags) {
        // Create StdDev Matrix
        Matrix<N3, N1> stdDevs =
            VecBuilder.fill(
                inputs.visionStdDevs[i * 3 + 0],
                inputs.visionStdDevs[i * 3 + 1],
                inputs.visionStdDevs[i * 3 + 2]);

        // Add to Drive
        Pose3d pose = inputs.estimatedPoses[i];

        if (!isValidPose(pose)) {
          continue;
        }

        drive.addVisionMeasurement(pose.toPose2d(), inputs.timestamps[i], stdDevs);
        validPoses.add(pose);
      }
    }
    return validPoses;
  }

  private boolean isValidPose(Pose3d pose) {
    return pose.getX() >= -VisionConstants.fieldBorderMargin
        && pose.getX() <= VisionConstants.fieldLength + VisionConstants.fieldBorderMargin
        && pose.getY() >= -VisionConstants.fieldBorderMargin
        && pose.getY() <= VisionConstants.fieldWidth + VisionConstants.fieldBorderMargin
        && Math.abs(pose.getZ()) <= VisionConstants.zMargin;
  }

  /** Pass the Simulation Truth Pose to the IOs (for Sim) */
  public void updateSimPose(Pose2d pose) {
    // If using ObjectDetectionIOSim, we need to update its robot pose reference
    if (frontMLIO instanceof ObjectDetectionIOSim) {
      ((ObjectDetectionIOSim) frontMLIO).updateRobotPose(pose);
    }
    // VisionIOSims use the shared VisionSystemSim updated in RobotContainer, so no need to call
    // them individually
    // unless we change that architecture.
  }
}
