package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

public class CameraIOLimelight implements CameraIO {

    protected String name_;
    protected Supplier<Rotation2d> rotationSupplier_;

    private Supplier<Long> lastUpdateSupplier_;
    private DoubleArrayEntry rawCornersNT_;
    private DoubleArrayEntry hardwareStatusNT_;

    public CameraIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
        name_ = name;
        rotationSupplier_ = rotationSupplier;

        lastUpdateSupplier_ = LimelightHelpers.getLimelightNTTableEntry(name_, "tl")::getLastChange;
        rawCornersNT_ = LimelightHelpers.getLimelightDoubleArrayEntry(name_, "tcornxy");
        hardwareStatusNT_ = LimelightHelpers.getLimelightDoubleArrayEntry(name_, "hw");
    }

    @Override
    public void updateInputs(CameraIOInputsAutoLogged inputs) {
        
        // Update Robot Orientation
        LimelightHelpers.SetRobotOrientation(name_, rotationSupplier_.get().getDegrees(), 0, 0, 0, 0, 0);
        
        // Results
        LimelightResults results = LimelightHelpers.getLatestResults(name_);
        
        double[] rawCorners = rawCornersNT_.get(new double[] {});
        double[] hardwareStatus = hardwareStatusNT_.get(new double[] {-1.0, -1.0, -1.0, -1.0});

        // Connected if not updated in one second
        inputs.connected = (Timer.getFPGATimestamp() - lastUpdateSupplier_.get()) < 1;

        // Status information
        inputs.fps = hardwareStatus[1];
        inputs.cpuTemp = hardwareStatus[0];

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
        if (rawCorners.length % 2 == 0) {
            for (int i = 0; i + 1 < rawCorners.length; i += 2) {
                corners.add(new Translation2d(rawCorners[i], rawCorners[i + 1]));
            }
        }

        // Fetch Fiducials
        for (LimelightTarget_Fiducial fid : results.targets_Fiducials) {
            fiducials.add(new Fiducial((int) fid.fiducialID, fid.ta, fid.tx, fid.ty));
        }

        inputs.rawCorners = corners.toArray(new Translation2d[0]);
        inputs.fiducials = fiducials.toArray(new Fiducial[0]);

        PoseEstimate estimateMegatag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name_);
        PoseEstimate estimateMegatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name_);

        if (estimateMegatag1 != null && estimateMegatag1.tagCount > 0) {
            poseEstimates.add(new PoseEstimation(
                estimateMegatag1.pose,
                estimateMegatag1.timestampSeconds,
                estimateMegatag1.avgTagDist,
                0.0,
                estimateMegatag1.tagCount,
                PoseEstimationType.MEGATAG1
            ));
        }
        
        if (estimateMegatag2 != null && estimateMegatag2.tagCount > 0) {
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
    public String getName() {
        return name_;
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
