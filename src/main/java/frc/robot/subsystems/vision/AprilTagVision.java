package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.vision.CameraIO.Fiducial;
import frc.robot.subsystems.vision.CameraIO.PoseEstimation;
import frc.robot.subsystems.vision.CameraIO.PoseEstimationType;

public class AprilTagVision extends SubsystemBase {

    private final PoseEstimateConsumer poseEstimateConsumer_;

    private final CameraIO[] io_;
    private final CameraIOInputsAutoLogged[] inputs_;

    private final Alert[] alerts_;

    public AprilTagVision(PoseEstimateConsumer poseEstimateConsumer, CameraIO... io) {
        poseEstimateConsumer_ = poseEstimateConsumer;

        io_ = io;

        inputs_ = new CameraIOInputsAutoLogged[io.length];
        alerts_ = new Alert[io.length];

        for (int i = 0; i < io.length; i++) {
            // Setup inputs for every camera
            inputs_[i] = new CameraIOInputsAutoLogged();

            // Setup alerts for every camera
            alerts_[i] = new Alert(
                getCameraName(i) + "\" is not connected!",
                AlertType.kWarning
            );
        }
    }

    @Override
    public void periodic() {

        // Update inputs for each camera
        for (int index = 0; index < io_.length; index++) {
            io_[index].updateInputs(inputs_[index]);
            Logger.processInputs("Vision/Camera" + index, inputs_[index]);
        }

        ArrayList<Pose3d> summaryTagPoses = new ArrayList<>();
        ArrayList<Pose2d> summaryPoses = new ArrayList<>();
        ArrayList<Pose2d> summaryAcceptedPoses = new ArrayList<>();
        ArrayList<Pose2d> summaryDeclinedPoses = new ArrayList<>();

        // Iterate cameras for logging and pose estimations.
        for (int cam = 0; cam < io_.length; cam++) {

            ArrayList<Pose3d> tagPoses = new ArrayList<>();
            ArrayList<Pose2d> estimatedPoses = new ArrayList<>();
            ArrayList<Pose2d> acceptedPoses = new ArrayList<>();
            ArrayList<Pose2d> declinedPoses = new ArrayList<>();

            // Activate disconnected alert.
            alerts_[cam].set(!inputs_[cam].connected);

            // Loop through visible tags.
            for (Fiducial fid : inputs_[cam].fiducials) {
                Optional<Pose3d> pose = FieldConstants.layout.getTagPose(fid.id());
                if (pose.isPresent()) {
                    tagPoses.add(pose.get());
                }
            }

            // Loop through pose estimations.
            for (PoseEstimation est : inputs_[cam].poseEstimates) {
                
                estimatedPoses.add(est.pose());

                if (isEstimationAcceptable(est)) {
                    acceptedPoses.add(est.pose());
                    integratePoseEstimate(est);
                } else {
                    declinedPoses.add(est.pose());
                }

            }

            // Add to summaries.
            summaryTagPoses.addAll(tagPoses);
            summaryPoses.addAll(estimatedPoses);
            summaryAcceptedPoses.addAll(acceptedPoses);
            summaryDeclinedPoses.addAll(declinedPoses);

            // Log camera-specific outputs.
            Logger.recordOutput(getCameraKey(cam, "TagPoses"), tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(getCameraKey(cam, "BotPoses/All"), estimatedPoses.toArray(new Pose2d[0]));
            Logger.recordOutput(getCameraKey(cam, "BotPoses/Accepted"), acceptedPoses.toArray(new Pose2d[0]));
            Logger.recordOutput(getCameraKey(cam, "BotPoses/Declined"), declinedPoses.toArray(new Pose2d[0]));

        }

        Logger.recordOutput("Vision/Summary/TagPoses", summaryTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/BotPoses/All", summaryPoses.toArray(new Pose2d[0]));
        Logger.recordOutput("Vision/Summary/BotPoses/Accepted", summaryAcceptedPoses.toArray(new Pose2d[0]));
        Logger.recordOutput("Vision/Summary/BotPoses/Declined", summaryDeclinedPoses.toArray(new Pose2d[0]));

    }

    @FunctionalInterface
    public static interface PoseEstimateConsumer {
        public void integrate(
            Pose2d robotPose,
            double timestampSecnds,
            Matrix<N3, N1> standardDeviations
        );
    }

    /**
     * Integrates a pose estimation with the PoseEstimator.
     * @param est
     * @deprecated This method is not fully implemented, it has no effect, and is simply a placeholder.
     */
    @Deprecated
    private void integratePoseEstimate(PoseEstimation est) {
        // TODO: Look into this calculation / into other calculations.
        double stdDevFactor =
            est.averageDist() * est.averageDist() / est.tagCount();
        double linearStdDev = 0.02 * stdDevFactor;
        double angularStdDev = 0.06 * stdDevFactor;

        Logger.recordOutput("Vision/PoseStdDev/Linear", linearStdDev);
        Logger.recordOutput("Vision/PoseStdDev/Angular", angularStdDev);

        if (est.type() == PoseEstimationType.MEGATAG2) {
          linearStdDev *= 0.5;
          angularStdDev *= Double.POSITIVE_INFINITY;
        }

        poseEstimateConsumer_.integrate(
            est.pose(),
            est.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
        );
    }

    /**
     * Decides whether or not a pose estimation is deemed acceptable to use in the PoseEstimator.
     * @param estimation
     * @return
     */
    private boolean isEstimationAcceptable(PoseEstimation estimation) {
        // Decline for any of the following reasons:

        if (estimation.tagCount() == 0) return false; // If there are no tags on the estimate.

        if (estimation.pose().getTranslation().getNorm() == 0) return false; // If the estimate is at the origin exactly (unrealistic).

        if (!isPoseOnField(estimation.pose())) return false; // The pose is not on the field.

        if (estimation.ambiguity() > 0.3) return false; // It is ambiguous (photonvision especially)

        // Otherwise, accept!
        return true;
    }

    /**
     * Whether or not the provided pose is within the field boundaries.
     * @param pose
     * @return
     */
    private boolean isPoseOnField(Pose2d pose) {
        boolean xIsValid = pose.getX() > 0.0 && pose.getX() < FieldConstants.layout.getFieldLength();
        boolean yIsValid = pose.getY() > 0.0 && pose.getY() < FieldConstants.layout.getFieldWidth();

        return xIsValid && yIsValid;
    }

    /**
     * Get the key of a camera-specific value.
     * @param index The camera to get the path of.
     * @param subtable The subtable within the camera to get.
     * @return The key to be used for logging.
     */
    private String getCameraKey(int index, String subtable) {
        return "Vision/"+ getCameraName(index) + "/" + subtable;
    }

    /**
     * Gets the name of the camera for logging
     * @param index
     * @return
     */
    private String getCameraName(int index) {
        return "Camera" + Integer.toString(index);
    }

}
