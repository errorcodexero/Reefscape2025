package frc.robot.subsystems.drive;

import java.util.Queue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.QuestNav;

public class GyroIOQuest implements GyroIO {

    private final QuestNav quest_;
    private final Queue<Double> yawPositionQueue_;
    private final Queue<Double> yawTimestampQueue_;

    public GyroIOQuest() {
        quest_ = new QuestNav();
        yawTimestampQueue_ = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue_ = PhoenixOdometryThread.getInstance().registerSignal(() -> getYaw().getRadians());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = quest_.getConnected();
        inputs.yawPosition = getYaw();

        inputs.odometryYawPositions = yawPositionQueue_
            .stream()
            .map(Rotation2d::fromRadians)
            .toArray(Rotation2d[]::new);

        inputs.odometryYawTimestamps = yawTimestampQueue_
            .stream()
            .mapToDouble((Double d) -> d)
            .toArray();
    }

    private Rotation2d getYaw() {
        return quest_.getPose().getRotation();
    }
    
}
