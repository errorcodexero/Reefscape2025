package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;

public interface TrackerIO {
    @AutoLog
    public static class TrackerInputs {
        public boolean connected = false;
        public double timestamp = 0.0;
        public double batteryPercent = 0.0;
        public long frameCount = 0;
        public boolean isTracking = false;
        public long trackingLostCount = 0;
        
        public Pose2d pose = new Pose2d();
        public Pose3d pose3d = new Pose3d();
        public Quaternion quaternion = new Quaternion();
    }

    public default void updateInputs(TrackerInputs inputs) {}
    public default void setPose(Pose2d pose) {}
}