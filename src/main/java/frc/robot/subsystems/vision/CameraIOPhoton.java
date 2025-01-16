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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;

public class CameraIOPhoton implements CameraIO {

    // Transform from robot to camera.
    protected static final Transform3d robotToCamera_ = new Transform3d(
    new Translation3d(-0.3549, 0, 0.16),
    new Rotation3d(0, Units.degreesToRadians(-40), Units.degreesToRadians(180))
    );
    
    protected final PhotonCamera camera_;
    private final PhotonPoseEstimator poseEstimator_;
    
    public CameraIOPhoton(String name) {
        // Setup camera
        camera_ = new PhotonCamera(name);
        
        // Setup pose stimator
        poseEstimator_ = new PhotonPoseEstimator(
            FieldConstants.layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera_
        );
    }
    
    @Override
    public void updateInputs(CameraIOInputsAutoLogged inputs) {
        
        inputs.connected = camera_.isConnected();

        PhotonPipelineResult result = camera_.getLatestResult();
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        
        // Simple setup
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
        
        Optional<EstimatedRobotPose> optionalPhotonEstimate = poseEstimator_.update(result);
        
        if (optionalPhotonEstimate.isPresent()) {
            inputs.poseEstimate[0] = new PoseEstimation(
                optionalPhotonEstimate.get().estimatedPose.toPose2d(),
                optionalPhotonEstimate.get().timestampSeconds,
                42.0,
                42.0,
                optionalPhotonEstimate.get().targetsUsed.size()
            );
        } else {
            inputs.poseEstimate = new PoseEstimation[] {};
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
