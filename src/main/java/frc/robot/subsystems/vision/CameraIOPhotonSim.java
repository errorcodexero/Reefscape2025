package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;

public class CameraIOPhotonSim extends CameraIOPhoton {
    private final VisionSystemSim sim_;
    private final PhotonCameraSim camSim_;
    private final SimCameraProperties camProps_;

    private final Supplier<Pose2d> robotPoseSupplier_;

    public CameraIOPhotonSim(String name, Transform3d robotToCamera, Supplier<Pose2d> robotPoseSupplier, boolean ll4) {
        super(name, robotToCamera);
        
        // Create vision sim
        sim_ = new VisionSystemSim(name);
        
        // Setup apriltags in sim
        sim_.addAprilTags(FieldConstants.layout);
        
        // Properties for camera simulation
        camProps_ = SimCameraProperties.LL2_960_720();

        // LL4 Calibration Estimate
        if (ll4) {
            camProps_.setCalibration(1280, 960, new Rotation2d(Degrees.of(90)));
        }
        
        // Setup camera sim
        camSim_ = new PhotonCameraSim(camera_, camProps_);

        sim_.addCamera(camSim_, robotToCamera_);

        if (Constants.getMode() != Mode.REAL) {
            Logger.recordOutput("CameraPoses/" + name, robotToCamera);
        }

        robotPoseSupplier_ = robotPoseSupplier;
    }

    @Override
    public void updateInputs(CameraIOInputsAutoLogged inputs) {
        sim_.update(robotPoseSupplier_.get());
        super.updateInputs(inputs);
    }
}
