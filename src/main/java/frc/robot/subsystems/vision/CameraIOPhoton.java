package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

public class CameraIOPhoton implements CameraIO {

    // Transform from robot to camera.
    protected final Transform3d robotToCamera_;
    protected final PhotonCamera camera_;
    
    private final PhotonPoseEstimator poseEstimator_;
    
    public CameraIOPhoton(String name, Transform3d robotToCamera) {
        // Setup camera
        camera_ = new PhotonCamera(name);
        robotToCamera_ = robotToCamera;
        
        // Setup pose stimator
        poseEstimator_ = new PhotonPoseEstimator(
            FieldConstants.layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera_
        );
    }
    
    @Override
    public void updateInputs(CameraIOInputsAutoLogged inputs) {
        
        // Connected Status
        inputs.connected = camera_.isConnected();
        
        // Camera Name
        inputs.name = camera_.getName();

        // Results
        PhotonPipelineResult result = camera_.getLatestResult();
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        
        if (bestTarget != null) {
            inputs.simpleID = bestTarget.getFiducialId();
            inputs.simpleX = bestTarget.getPitch();
            inputs.simpleY = bestTarget.getYaw();
            inputs.simpleArea = bestTarget.getArea();
            inputs.simpleValid = true;
        } else {
            inputs.simpleID = 0;
            inputs.simpleX = 0.0;
            inputs.simpleY = 0.0;
            inputs.simpleArea = 0.0;
            inputs.simpleValid = false;
        }
        
        // Target information to fill
        ArrayList<Translation2d> cornerCoords = new ArrayList<>();
        ArrayList<Fiducial> fiducials = new ArrayList<>();
        
        // Get target information
        for (PhotonTrackedTarget target : result.getTargets()) {
            
            for (TargetCorner corner : target.getDetectedCorners()) {
                cornerCoords.add(new Translation2d(corner.x, corner.y));
            }
            
            fiducials.add(new Fiducial(
                target.getFiducialId(),
                target.getArea(),
                target.getPitch(),
                target.getYaw()
            ));
        }
        
        inputs.rawCorners = cornerCoords.toArray(new Translation2d[0]);
        inputs.fiducials = fiducials.toArray(new Fiducial[0]);

        // Calculate Average Tag Distance and Ambiguity
        double averageTagDist = 0.0;

        @SuppressWarnings("unused")
        double averageAmbiguity = 0.0;
        
        for (PhotonTrackedTarget target : result.targets) {
            averageTagDist += target.getBestCameraToTarget().getTranslation().getNorm();
            averageAmbiguity += target.getPoseAmbiguity();
        }
        averageTagDist /= result.targets.size();
        averageAmbiguity /= result.targets.size();
        
        Optional<EstimatedRobotPose> optionalPhotonEstimate = poseEstimator_.update(result);
        
        if (optionalPhotonEstimate.isPresent()) {
            inputs.poseEstimates[0] = new PoseEstimation(
                optionalPhotonEstimate.get().estimatedPose.toPose2d(),
                optionalPhotonEstimate.get().timestampSeconds,
                averageTagDist,
                optionalPhotonEstimate.get().targetsUsed.size(),
                PoseEstimationType.PHOTON_MULTITAG
            );
        } else {
            inputs.poseEstimates = new PoseEstimation[] {};
        }
    }
    
    @Override
    public void forceBlink() {
        camera_.setLED(VisionLEDMode.kBlink);
    }
    
    @Override
    public void forceOff() {
        camera_.setLED(VisionLEDMode.kOff);
    }
    
    @Override
    public void forceOn() {
        camera_.setLED(VisionLEDMode.kOn);
    }
    
    @Override
    public void resetLed() {
        camera_.setLED(VisionLEDMode.kDefault);
    }
}
