package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import gg.questnav.questnav.QuestNav;

public class TrackerIOQuest implements TrackerIO {

    private final QuestNav quest = new QuestNav();

    @Override
    public void updateInputs(TrackerInputs inputs) {

        quest.commandPeriodic();

        inputs.batteryPercent = quest.getBatteryPercent().orElse(-1);
        inputs.connected = quest.isConnected();
        inputs.isTracking = quest.isTracking();

        inputs.frameCount = quest.getFrameCount().orElse(-1);
        inputs.trackingLostCount = quest.getTrackingLostCounter().orElse(-1);

        inputs.unreadFrames = quest.getAllUnreadPoseFrames();
    }
    
    @Override
    public void setPose(Pose2d pose) {
        quest.setPose(pose);
    }
    
}