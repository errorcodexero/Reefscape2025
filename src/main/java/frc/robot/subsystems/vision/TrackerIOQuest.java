package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import gg.questnav.questnav.QuestNav;

public class TrackerIOQuest implements TrackerIO {
    
    private static final Transform2d robotToQuest = new Transform2d(
        Inches.of(12),
        Inches.of(9.25),
        Rotation2d.kZero
    );

    private final QuestNav quest = new QuestNav();

    @Override
    public void updateInputs(TrackerInputs inputs) {

        quest.commandPeriodic();

        inputs.batteryPercent = quest.getBatteryPercent();
        inputs.connected = quest.isConnected();
        inputs.isTracking = quest.isTracking();

        inputs.frameCount = quest.getFrameCount();
        inputs.trackingLostCount = quest.getTrackingLostCounter();
        
        inputs.timestamp = quest.getDataTimestamp();
        inputs.pose = quest.getPose().transformBy(robotToQuest.inverse());
    }
    
    @Override
    public void setPose(Pose2d pose) {
        quest.setPose(pose.transformBy(robotToQuest));
    }
    
}