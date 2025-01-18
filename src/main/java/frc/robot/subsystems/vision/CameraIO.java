package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

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
        PoseEstimationType type
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

    }

    /**
     * Updates the inputs object with values from the hardware.
     * @param inputs The inputs to update
     */
    public default void updateInputs(CameraIOInputsAutoLogged inputs) {};

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
