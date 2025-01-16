package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.vision.CameraIO.Fiducial;

public class AprilTagVision extends SubsystemBase {

    private final CameraIO[] io_;
    private final CameraIOInputsAutoLogged[] inputs_;

    private final Alert[] alerts_;

    public AprilTagVision(Consumer<Pose2d> poseConsumer, CameraIO... io) {
        io_ = io;

        inputs_ = new CameraIOInputsAutoLogged[io.length];
        alerts_ = new Alert[io.length];

        for (int i = 0; i < io.length; i++) {
            // Setup inputs for every camera
            inputs_[i] = new CameraIOInputsAutoLogged();

            // Setup alerts for every camera
            alerts_[i] = new Alert(
                "Apriltag Vision Camera " + i + " is not connected!",
                AlertType.kError
            );
        }

    }

    @Override
    public void periodic() {

        // Update inputs for each camera
        for (int index = 0; index < io_.length; index++) {
            io_[index].updateInputs(inputs_[index]);
            Logger.processInputs("Camera" + index, inputs_[index]);
        }

        ArrayList<Pose3d> summaryTagPoses = new ArrayList<>();

        for (int cam = 0; cam < io_.length; cam++) {

            alerts_[cam].set(!inputs_[cam].connected);

            ArrayList<Pose3d> tagPoses = new ArrayList<>();

            for (Fiducial fid : inputs_[cam].fiducials) {
                Optional<Pose3d> pose = FieldConstants.layout.getTagPose(fid.id());
                if (pose.isPresent()) {
                    tagPoses.add(pose.get());
                }
            }

        }

    }

    /**
     * Get all the Pose3d tag poses from an array of Fiducial objects.
     * @param fids
     * @return An array of tag poses
     */
    private void getTagPoses(Fiducial[] fids) {

    }

}
