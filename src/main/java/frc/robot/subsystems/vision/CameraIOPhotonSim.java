package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldConstants;

public class CameraIOPhotonSim extends CameraIOPhoton {
    private final VisionSystemSim sim_;
    private final PhotonCameraSim camSim_;
    private final SimCameraProperties camProps_;

    private final Supplier<Pose2d> robotPoseSupplier_;

    public CameraIOPhotonSim(String name, Supplier<Pose2d> robotPoseSupplier) {
        super(name);
        
        // Create vision sim
        sim_ = new VisionSystemSim(name);
        
        // Setup apriltags in sim
        sim_.addAprilTags(FieldConstants.layout);
        
        // Properties for camera simulation
        camProps_ = SimCameraProperties.LL2_960_720();
        
        // Setup camera sim
        camSim_ = new PhotonCameraSim(camera_, camProps_);
        camSim_.enableDrawWireframe(true);
        
        sim_.addCamera(camSim_, robotToCamera_);

        robotPoseSupplier_ = robotPoseSupplier;
    }

    @Override
    public void updateInputs(CameraIOInputsAutoLogged inputs) {
        sim_.update(robotPoseSupplier_.get());
        super.updateInputs(inputs);
    }
}
