package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class CameraIOPhoton implements CameraIO {

    // Transform from robot to camera.
    protected final Transform3d robotToCamera_;
    protected final PhotonCamera camera_;

    public CameraIOPhoton(String name, Transform3d robotToCamera) {
        // Setup camera
        camera_ = new PhotonCamera(name);
        robotToCamera_ = robotToCamera;
    }
    
    @Override
    public void updateInputs(CameraIOInputsAutoLogged inputs) {
        
        // Connected Status
        inputs.connected = camera_.isConnected();
        
        // Camera Name
        inputs.name = camera_.getName();

        // Results
        List<PhotonPipelineResult> results = camera_.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {

            if (!result.hasTargets()) {
                inputs.simpleID = 0;
                inputs.simpleX = 0.0;
                inputs.simpleY = 0.0;
                inputs.simpleArea = 0.0;
                inputs.simpleValid = false;

                inputs.poseEstimates = new PoseEstimation[] {};
                inputs.fiducials = new Fiducial[] {};
                inputs.rawCorners = new Translation2d[] {};

                continue;
            }

            // Get best target

            PhotonTrackedTarget bestTarget = result.getBestTarget();
            
            // Target information to fill

            ArrayList<Translation2d> cornerCoords = new ArrayList<>();
            ArrayList<Fiducial> fiducials = new ArrayList<>();
            ArrayList<PoseEstimation> poseEstimates = new ArrayList<>();
            
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

            // Calculate Average Tag Distance and Ambiguity

            double averageTagDist = 0.0;

            for (PhotonTrackedTarget target : result.targets) {
                averageTagDist += target.getBestCameraToTarget().getTranslation().getNorm();
            }

            averageTagDist /= result.targets.size();

            // Get Pose Estimates

            Optional<MultiTargetPNPResult> multitagResult = result.multitagResult;
            
            if (multitagResult.isPresent()) {
                Transform3d fieldToCamera = multitagResult.get().estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera_.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                poseEstimates.add(new PoseEstimation(
                    robotPose.toPose2d(),
                    result.getTimestampSeconds(),
                    averageTagDist,
                    multitagResult.get().estimatedPose.ambiguity,
                    multitagResult.get().fiducialIDsUsed.size(),
                    PoseEstimationType.PHOTON_MULTITAG
                ));
            }

            inputs.simpleID = bestTarget.getFiducialId();
            inputs.simpleX = bestTarget.getPitch();
            inputs.simpleY = bestTarget.getYaw();
            inputs.simpleArea = bestTarget.getArea();
            inputs.simpleValid = true;

            inputs.poseEstimates = poseEstimates.toArray(new PoseEstimation[0]);
            inputs.rawCorners = cornerCoords.toArray(new Translation2d[0]);
            inputs.fiducials = fiducials.toArray(new Fiducial[0]);
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
