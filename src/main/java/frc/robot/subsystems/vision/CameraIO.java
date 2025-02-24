package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.CameraIOLimelight4.IMUMode;

/**
 * An interface encapsulating a single camera in a vision system.
 */
public interface CameraIO {

    public static enum PoseEstimationType {
        MEGATAG1,
        MEGATAG2,
        PHOTON_MULTITAG
    }

    public static record PoseEstimation(
        Pose2d pose,
        double timestamp,
        double averageDist,
        double ambiguity,
        int tagCount,
        PoseEstimationType type,
        String cameraName
    ) {};

    public static record Fiducial(
        int id,
        double area,
        double x,
        double y
    ) {};
    
    @AutoLog
    public static class CameraIOInputs {
        
        public boolean connected = false;
        public String name = "";

        public int simpleID = 0;
        public double simpleX = 0.0;
        public double simpleY = 0.0;
        public double simpleArea = 0.0;
        public boolean simpleValid = false;
        
        public Translation2d[] rawCorners = new Translation2d[] {};
        
        public Fiducial[] fiducials = new Fiducial[] {};
        public PoseEstimation[] poseEstimates = new PoseEstimation[] {};

        public IMUMode imuMode = IMUMode.NONE;
        public Rotation2d imuRobotYaw = Rotation2d.kZero;
        public int frameSkipThrottle = 0;

        public double cpuTemp = -1.0;
        public double fps = -1.0;
        
    }

    /**
     * Updates the inputs object with values from the hardware.
     * @param inputs The inputs to update
     */
    public default void updateInputs(CameraIOInputsAutoLogged inputs) {};

    /**
     * Gets a human-readable name of the camera. Mostly for alerts.
     * @return The human-readable name of this camera.
     * @apiNote THIS SHOULD NEVER BE USED FOR ROBOT CONTROL FLOW! THIS IS ONLY FOR ALERT READABILITY!
     */
    public default String getName() { return "Camera"; }

    /**
     * Forces the indicator light on the limelight to be off.
     */
    public default void forceOff() {};

    /**
     * Forces the indicator light on the limelight to blink.
     */
    public default void forceBlink() {};

    /**
     * Forces the indicator light on the limelight to be on.
     */
    public default void forceOn() {};

    /**
     * Resets the indicator light on the limelight to be controlled by its own software/pipelines.
     */
    public default void resetLed() {};
    
}
