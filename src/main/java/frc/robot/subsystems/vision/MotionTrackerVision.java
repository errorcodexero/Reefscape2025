
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import gg.questnav.questnav.PoseFrame;

public class MotionTrackerVision extends SubsystemBase {

    private static final Matrix<N3, N1> stdDevs = VecBuilder.fill(
        0.0001,
        0.0001,
        0.0001
    );
    
    private boolean useQuestNav = false;
    private static final boolean calibration = false;
    private static final boolean useQueuedFrames = true;

    private static final Transform2d robotToQuest = new Transform2d(
        !calibration ? Centimeters.of(24.275) : Meters.zero(),
        !calibration ? Centimeters.of(26.2) : Meters.zero(),
        new Rotation2d(Degrees.of(45))
    );

    // x: -0.216 y: -0.261

    // time: 565-593
    // time: 794-831
    private final TrackerIO io_;
    private final TrackerInputsAutoLogged inputs_;

    private final Alert disconnectedAlert_ = new Alert("Motion Tracker Disconnected!", AlertType.kError);
    private final Alert lowBatteryAlert_ = new Alert("Quest battery is low!", AlertType.kWarning);
    private final Alert notInitedAlert_ = new Alert("The Quest is not initialized or initializing!!", AlertType.kWarning);
    private final Alert initAlert_ = new Alert("Quest has been initialized!", AlertType.kInfo);

    private final PoseEstimateConsumer estimateConsumer_;

    // Whether or not the quest has been zeroed to the field.
    private boolean zeroed = false;

    public MotionTrackerVision(
        TrackerIO io,
        PoseEstimateConsumer estimateConsumer
    ) {
        io_ = io;
        inputs_ = new TrackerInputsAutoLogged();
        estimateConsumer_ = estimateConsumer;
    }

    public Command zero(Drive drive, AprilTagVision vision) {
        return Commands.sequence(
            runOnce(() -> zeroed = false),
            Commands.waitUntil(() -> DriverStation.getAlliance().isPresent()),
            drive.runOnce(() ->
                drive.setPose(
                    DriverStation.getAlliance().orElseThrow() == Alliance.Blue
                        ? new Pose2d(0, 0, Rotation2d.k180deg)
                        : Pose2d.kZero
                )
            ),
            Commands.waitTime(Seconds.of(0.5)),
            Commands.waitUntil(() -> vision.getTagCount() > 0),
            runOnce(() -> {
                setPose(drive.getPose());
                zeroed = true;
            })
        );
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs("MotionTrackerVision", inputs_);

        boolean disconnected = !inputs_.connected;
        disconnectedAlert_.set(disconnected);
        lowBatteryAlert_.set(inputs_.batteryPercent < 20);
        notInitedAlert_.set(!zeroed);
        initAlert_.set(zeroed);

        if (
            disconnected ||
            !inputs_.isTracking ||
            RobotState.isDisabled() ||
            !zeroed ||
            !useQuestNav
        ) return;

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

    private void setPose(Pose2d pose) {
        io_.setPose(pose.transformBy(robotToQuest));
    }

    public boolean isConnected() {
        return inputs_.connected;
    }

}