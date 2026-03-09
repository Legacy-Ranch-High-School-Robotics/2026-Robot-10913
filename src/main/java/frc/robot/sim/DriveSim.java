package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.robot.subsystems.MAXSwerveModule;

/**
 * The maple-sim wrapper for the drivetrain.
 * Handles field collisions and realistic odometry drift (wheel slip) natively through maple-sim.
 */
public class DriveSim {
    private final MAXSwerveModule m_frontLeft;
    private final MAXSwerveModule m_frontRight;
    private final MAXSwerveModule m_rearLeft;
    private final MAXSwerveModule m_rearRight;
    private final ADIS16470_IMU m_gyro;

    private Pose2d m_actualPose = new Pose2d(); // Ground truth pose from maple-sim

    public DriveSim(MAXSwerveModule frontLeft, MAXSwerveModule frontRight, 
                    MAXSwerveModule rearLeft, MAXSwerveModule rearRight,
                    ADIS16470_IMU gyro) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
        m_gyro = gyro;

        // TODO: Initialize maple-sim physics engine
        // - configure the robot's weight
        // - configure bumper size
        // - configure wheel coefficient of friction
    }

    /**
     * Steps the simulation forward.
     */
    public void update() {
        // 1. Read the voltage/percent output from the motor objects (m_frontLeft, etc.)
        
        // 2. Feed those voltages into the maple-sim physics model
        
        // 3. Step the maple-sim physics engine forward by 20ms
        
        // 4. Extract the resulting slipping wheel velocities and chassis heading from maple-sim,
        //    and manually inject those values into the simulated WPILib encoders and simulated Gyro.
        
        // Update the ground truth pose (e.g., this would be obtained from maple-sim)
        // m_actualPose = maplesim.getPose();
    }

    /**
     * @return Actual Pose (Ground Truth) managed entirely by maple-sim.
     */
    public Pose2d getActualPose() {
        return m_actualPose;
    }
}
