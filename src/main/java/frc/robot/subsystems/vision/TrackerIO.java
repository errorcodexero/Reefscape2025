package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import gg.questnav.questnav.PoseFrame;

public interface TrackerIO {
    @AutoLog
    public static class TrackerInputs {
        public boolean connected = false;
        public double timestamp = 0.0;
        public double batteryPercent = 0.0;
        public long frameCount = 0;
        public boolean isTracking = false;
        public long trackingLostCount = 0;
        
        public PoseFrame[] unreadFrames = {};
    }

    public default void updateInputs(TrackerInputs inputs) {}
    public default void setPose(Pose2d pose) {}
}