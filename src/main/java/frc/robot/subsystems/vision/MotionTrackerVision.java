
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;

public class MotionTrackerVision extends SubsystemBase {

    private static final Matrix<N3, N1> stdDevs = VecBuilder.fill(
        0.01,
        0.01,
        0.01
    );

    private static final Transform2d robotToQuest = new Transform2d(
        Inches.of(12),
        Inches.of(9.25),
        Rotation2d.kZero
    );

    private static final boolean useQueuedFrames = true;

    private final TrackerIO io_;
    private final TrackerInputsAutoLogged inputs_;

    private final Alert disconnectedAlert_ = new Alert("Motion Tracker Disconnected!", AlertType.kError);

    private final Alert lowBatteryAlert_ = new Alert("Quest battery is low!", AlertType.kWarning);

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
        
        lowBatteryAlert_.set(inputs_.batteryPercent < 20);

        if (disconnected || !inputs_.isTracking || RobotState.isDisabled()) return;

        if (useQueuedFrames) {
            for (PoseFrame frame : inputs_.unreadFrames) {
                Pose2d robotPose = frame.questPose().transformBy(robotToQuest.inverse());
                double timestamp = frame.dataTimestamp();
                estimateConsumer_.integrate(robotPose, timestamp, stdDevs);
            }
        } else {
            PoseFrame frame = inputs_.unreadFrames[inputs_.unreadFrames.length - 1];
            estimateConsumer_.integrate(
                frame.questPose().transformBy(robotToQuest.inverse()),
                frame.dataTimestamp(),
                stdDevs
            );
        }
    }

    public void setPose(Pose2d pose) {
        io_.setPose(pose.transformBy(robotToQuest));
    }

    public boolean isConnected() {
        return inputs_.connected;
    }

}