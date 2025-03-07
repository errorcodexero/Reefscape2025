package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.CameraIOLimelight4.IMUMode;

public class VisionConstants {

    // Limelight Names
    public static final String frontLimelightName = "limelight-front";
    public static final String backLimelightName = "limelight-back";
    public static final String leftLimelightName = "limelight-left";

    // Exclusively use the front apriltag.
    public static final boolean onlyUseFront = false;

    // Transforms
    public static final Transform3d frontTransform = new Transform3d(
        new Translation3d(Meters.of(0.1975), Meters.of(-0.2032), Meters.of(0.205)),
        new Rotation3d(Degrees.zero(), Degrees.of(-20), Degrees.zero())
    );

    public static final Transform3d backTransform = new Transform3d(
        new Translation3d(Meters.of(-0.3159252), Meters.of(-0.0508), Meters.of(0.1760474)),
        new Rotation3d(Degrees.zero(), Degrees.of(-20), Degrees.of(180))
    );

    public static final Transform3d leftTransform = new Transform3d(
        new Translation3d(Meters.of(0.077343), Meters.of(0.322961), Meters.of(0.1825752)),
        new Rotation3d(Degrees.zero(), Degrees.of(-20), Degrees.of(100))
    );

    // Odometry Filtering Configuration
    public static final int minimumTagCount = 1;
    public static final double maximumAmbiguity = 0.3; // For Photonvision Sim

    // Standard Deviation Factors, (we need to talk about how stddev is calculated, dont change until then)
    public static final double baseLinearStdDev = 0.02;
    public static final double baseAngularStdDev = 0.06;
    public static final double megatag2Factor = 0.5;

    // LL4 Config
    public static final boolean useIMU = false; // Whether or not to use the internal IMU
    public static final IMUMode enabledIMUMode = IMUMode.ASSIST_EXTERNAL; // What mode to use on the internal IMU
    
    public static final boolean regulateThrottle = false; // Whether or not we use LL4 throttle to regulate temperature
    public static final int numSkippedFramesEnabled = 0; // How many frames to skip in enabled, (probably 0 but you might want to?)
    public static final int numSkippedFramesDisabled = 100; // Regulate temperature by skipping frames in disabled

}