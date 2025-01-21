package frc.robot.subsystems.vision;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.LimelightHelpers.RawDetection;

public class CameraIOLimelight implements CameraIO {

    private String name_;

    public CameraIOLimelight(String name) {
        name_ = name;
    }

    @Override
    public void updateInputs(CameraIOInputsAutoLogged inputs) {

        LimelightResults results = LimelightHelpers.getLatestResults(name_);
        RawDetection[] detections = LimelightHelpers.getRawDetections(name_);

        // Connected if not updated in one second
        inputs.connected = (Timer.getFPGATimestamp() - results.timestamp_RIOFPGA_capture) < 1;

        // Camera name
        inputs.name = name_;

        // Simple Data
        inputs.simpleID = (int) LimelightHelpers.getFiducialID(name_);
        inputs.simpleArea = LimelightHelpers.getTA(name_);
        inputs.simpleValid = LimelightHelpers.getTV(name_);
        inputs.simpleX = LimelightHelpers.getTX(name_);
        inputs.simpleY = LimelightHelpers.getTY(name_);

        ArrayList<Translation2d> corners = new ArrayList<>();
        ArrayList<Fiducial> fiducials = new ArrayList<>();
        ArrayList<PoseEstimation> poseEstimates = new ArrayList<>();

        // Fetch Raw Corners
        for (RawDetection detection : detections) {
            corners.add(new Translation2d(detection.corner0_X, detection.corner0_Y));
            corners.add(new Translation2d(detection.corner1_X, detection.corner1_Y));
            corners.add(new Translation2d(detection.corner2_X, detection.corner2_Y));
            corners.add(new Translation2d(detection.corner3_X, detection.corner3_Y));
        }

        // Fetch Fiducials
        for (LimelightTarget_Fiducial fid : results.targets_Fiducials) {
            fiducials.add(new Fiducial((int) fid.fiducialID, fid.ta, fid.tx, fid.ty));
        }

        inputs.rawCorners = corners.toArray(new Translation2d[0]);
        inputs.fiducials = fiducials.toArray(new Fiducial[0]);

        PoseEstimate estimateMegatag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name_);
        PoseEstimate estimateMegatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name_);

        if (estimateMegatag1 != null) {
            poseEstimates.add(new PoseEstimation(
                estimateMegatag1.pose,
                estimateMegatag1.timestampSeconds,
                estimateMegatag1.avgTagDist,
                0.0,
                estimateMegatag1.tagCount,
                PoseEstimationType.MEGATAG1
            ));
        }
        
        if (estimateMegatag2 != null) {
            poseEstimates.add(new PoseEstimation(
                estimateMegatag2.pose,
                estimateMegatag2.timestampSeconds,
                estimateMegatag2.avgTagDist,
                0.0,
                estimateMegatag2.tagCount,
                PoseEstimationType.MEGATAG2
            ));
        }

        inputs.poseEstimates = poseEstimates.toArray(new PoseEstimation[0]);

    }

    @Override
    public void forceBlink() {
        LimelightHelpers.setLEDMode_ForceBlink(name_);
    }

    @Override
    public void forceOff() {
        LimelightHelpers.setLEDMode_ForceOff(name_);
    }

    @Override
    public void forceOn() {
        LimelightHelpers.setLEDMode_ForceOn(name_);
    }

    @Override
    public void resetLed() {
        LimelightHelpers.setLEDMode_PipelineControl(name_);
    }

}
