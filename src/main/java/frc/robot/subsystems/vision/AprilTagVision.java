package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.vision.CameraIO.Fiducial;
import frc.robot.subsystems.vision.CameraIO.PoseEstimation;
import frc.robot.subsystems.vision.CameraIO.PoseEstimationType;

public class AprilTagVision extends SubsystemBase {

    private final static Distance kMaxTagDistance = Meters.of(2.0) ;

    public static enum IntegrationBehavior {
        ONLY_NEAREST,
        ALL
    }

    private final PoseEstimateConsumer poseEstimateConsumer_;

    private final CameraIO[] io_;
    private final CameraIOInputsAutoLogged[] inputs_;

    private final Alert[] alerts_;

    private boolean enabled_; // Whether or not vision pose estimation is enabled

    public AprilTagVision(PoseEstimateConsumer poseEstimateConsumer, CameraIO... io) {
        poseEstimateConsumer_ = poseEstimateConsumer;

        io_ = io;
        enabled_ = true;

        inputs_ = new CameraIOInputsAutoLogged[io.length];
        alerts_ = new Alert[io.length];

        for (int i = 0; i < io.length; i++) {
            // Setup inputs for every camera
            inputs_[i] = new CameraIOInputsAutoLogged();

            // Setup alerts for every camera
            alerts_[i] = new Alert(
                "Camera " + i + ", \"" + io_[i].getName() + "\" is not connected!",
                AlertType.kWarning
            );
        }

    }

    /**
     * Sets whether or not the vision pose estimates get merged with the drivebase.
     * @param enabled
     */
    public void setEnabled(boolean enabled) {
        enabled_ = enabled;
    }

    /**
     * Command that sets whether or not the vision pose estimates get merged with the drivebase.
     * @param enabled
     */
    public Command setEnabledCommand(boolean enabled) {
        return runOnce(() -> setEnabled(enabled));
    }

    @Override
    public void periodic() {
            
        // Update inputs for each camera
        for (int index = 0; index < io_.length; index++) {
            io_[index].updateInputs(inputs_[index]);
            Logger.processInputs("Vision/Camera" + index, inputs_[index]);
        }

        ArrayList<Pose3d> summaryTagPoses = new ArrayList<>();
        ArrayList<PoseEstimation> poseEstimates = new ArrayList<>();

        // Iterate cameras for logging and pose estimations.
        for (int cam = 0; cam < io_.length; cam++) {

            // Activate disconnected alert.
            alerts_[cam].set(!inputs_[cam].connected);

            // Skip this camera if it is not connected.
            if (!inputs_[cam].connected) continue;

            ArrayList<Pose3d> tagPoses = new ArrayList<>();

            // Loop through visible tags.
            for (Fiducial fid : inputs_[cam].fiducials) {
                Optional<Pose3d> pose = FieldConstants.layout.getTagPose(fid.id());
                if (pose.isPresent()) {
                    tagPoses.add(pose.get());
                }
            }

            PoseEstimation est = inputs_[cam].poseEstimate;
            poseEstimates.add(est);

            // Add to summaries.
            summaryTagPoses.addAll(tagPoses);

            // Log camera-specific outputs.
            Logger.recordOutput(getCameraKey(cam, "TagPoses"), tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(getCameraKey(cam, "BotPose"), est.pose());
        }

        // Integrate pose estimates

        ArrayList<PoseEstimation> acceptedEstimates = new ArrayList<>();
        ArrayList<PoseEstimation> declinedEstimates = new ArrayList<>();

        for (PoseEstimation est : poseEstimates) {
            if (isEstimationAcceptable(est)) {
                acceptedEstimates.add(est);
            } else {
                declinedEstimates.add(est);
            }
        }

        switch (VisionConstants.integrationBehavior) {
            case ONLY_NEAREST -> {
                Optional<PoseEstimation> est = findMinDistanceEstimate(acceptedEstimates);

                if (est.isPresent()) {
                    integratePoseEstimate(est.orElseThrow());
                    Logger.recordOutput("Vision/Summary/EstimateUsing", est.orElseThrow());
                }
                
            }
            default -> {
                for (PoseEstimation est : acceptedEstimates) {
                    integratePoseEstimate(est);
                }
            }
        }

        Logger.recordOutput("Vision/Summary/TagPoses", summaryTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/BotPoses/All", estimateListToPoseArray(poseEstimates));
        Logger.recordOutput("Vision/Summary/BotPoses/Accepted", estimateListToPoseArray(acceptedEstimates));
        Logger.recordOutput("Vision/Summary/BotPoses/Declined", estimateListToPoseArray(declinedEstimates));

        Logger.recordOutput("Vision/PoseEstimatesEnabled", enabled_);

    }

    @FunctionalInterface
    public static interface PoseEstimateConsumer {
        public void integrate(
            Pose2d robotPose,
            double timestampSecnds,
            Matrix<N3, N1> standardDeviations
        );

        public static PoseEstimateConsumer ignore() {
            return (Pose2d robotPose, double timestampSecnds, Matrix<N3, N1> standardDeviations) -> {};
        }
    }

    /**
     * Integrates a pose estimation with the PoseEstimator.
     * @param est
     */
    private void integratePoseEstimate(PoseEstimation est) {
        double linearStdDev = VisionConstants.megatag2Factor;
        double angularStdDev = Double.POSITIVE_INFINITY;

        poseEstimateConsumer_.integrate(
            est.pose(),
            est.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
        );
    }

    /**
     * Decides whether or not a pose estimation is deemed acceptable to use in the PoseEstimator.
     * @param estimation
     * @return Whether or not to integrate the pose.
     */
    private boolean isEstimationAcceptable(PoseEstimation estimation) {
        // Decline for any of the following reasons:

        if (!enabled_) return false; // If vision pose estimation is disabled.

        if (!estimation.valid()) return false; // If its not a valid pose estimate

        if (estimation.tagCount() == 0) return false; // If there are no tags on the estimate.

        if (estimation.type() == PoseEstimationType.MEGATAG1) return false; // If it is using Megatag1.

        if (estimation.tagCount() < VisionConstants.minimumTagCount) return false; // If there are less than the configured minimum.

        if (estimation.pose().getTranslation().getNorm() == 0) return false; // If the estimate is at the origin exactly (unrealistic).

        if (!isPoseOnField(estimation.pose())) return false; // The pose is not on the field.

        if (estimation.ambiguity() > VisionConstants.maximumAmbiguity) return false; // It is ambiguous (photonvision especially)

        return true;
    }

    /**
     * Finds the minimum distance estimate from an arraylist of estimates.
     * @param estimates
     * @return The estimate with the least minimum distance
     */
    private Optional<PoseEstimation> findMinDistanceEstimate(ArrayList<PoseEstimation> estimates) {
        if (estimates.size() <= 0) return Optional.empty();

        PoseEstimation minDistance = estimates.get(0);

        for (int i = 1; i < estimates.size(); i++) {
            PoseEstimation est = estimates.get(i);
            if (est.averageDist() < minDistance.averageDist()) {
                minDistance = est;
            }
        }

        if (Meters.of(minDistance.averageDist()).gt(kMaxTagDistance)) {
            //
            // If the tag is greater than a given distance away, ignore it.  This is useful
            // for station collect during autonomous.
            //
            return Optional.empty() ;
        }

        return Optional.of(minDistance);
    }

    private Pose2d[] estimateListToPoseArray(List<PoseEstimation> estimates) {
        return estimates.stream().map(PoseEstimation::pose).toList().toArray(new Pose2d[0]);
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
     * Gets the name of the camera subtable for logging
     * @param index
     * @return
     */
    private String getCameraName(int index) {
        return "Camera" + Integer.toString(index);
    }

}
