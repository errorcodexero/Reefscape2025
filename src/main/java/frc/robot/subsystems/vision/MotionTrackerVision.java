
package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotionTrackerVision extends SubsystemBase {

    private static final Matrix<N3, N1> stdDevs = VecBuilder.fill(
        0.01,
        0.01,
        0.01
    );

    private final TrackerIO io_;
    private final TrackerInputsAutoLogged inputs_;

    private final Alert disconnectedAlert_ = new Alert("Motion Tracker Disconnected!", AlertType.kError);

    private final PoseEstimateConsumer estimateConsumer_;

    public MotionTrackerVision(TrackerIO io, PoseEstimateConsumer estimateConsumer) {
        io_ = io;
        inputs_ = new TrackerInputsAutoLogged();
        estimateConsumer_ = estimateConsumer;
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("MotionTrackerVision", inputs_);

        boolean disconnected = !inputs_.connected;
        disconnectedAlert_.set(disconnected);

        if (disconnected || !inputs_.isTracking) return;

        estimateConsumer_.integrate(inputs_.pose, inputs_.timestamp, stdDevs);
    }

    public void setPose(Pose2d pose) {
        io_.setPose(pose);
    }

    public Pose2d getPose() {
        return inputs_.pose;
    }

    public boolean isConnected() {
        return inputs_.connected;
    }

}