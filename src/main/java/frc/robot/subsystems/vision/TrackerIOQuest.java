package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class TrackerIOQuest implements TrackerIO {

    private final QuestNav quest = new QuestNav();

    @Override
    public void updateInputs(TrackerInputs inputs) {

        quest.cleanupResponses();
        quest.processHeartbeat();

        inputs.connected = quest.getConnected();
        inputs.timestamp = quest.getTimestamp();
        inputs.batteryPercent = quest.getBatteryPercent();
        inputs.frameCount = quest.getFrameCount();
        inputs.isTracking = quest.getTrackingStatus();
        inputs.trackingLostCount = quest.getTrackingLostCounter();

        inputs.pose = quest.getPose();
        inputs.pose3d = quest.getPose3d();
        inputs.quaternion = quest.getQuaternion();
    }
    
    @Override
    public void setPose(Pose2d pose) {
        quest.setPose(pose);
    }
    
}