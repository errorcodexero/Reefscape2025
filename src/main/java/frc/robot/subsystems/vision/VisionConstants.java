package frc.robot.subsystems.vision;

public class VisionConstants {

    // Limelight Names
    public static final String frontLimelightName = "frontlimelight";
    public static final String backLimelightName = "backlimelight";
    public static final String leftLimelightName = "leftlimelight";

    // Odometry Filtering Configuration
    
    public static final int minimumTagCount = 1;
    public static final double maximumAmbiguity = 0.3; // For Photonvision Sim

    // Standard Deviation Factors, (we need to talk about how stddev is calculated, dont change until then)
    public static final double baseLinearStdDev = 0.02;
    public static final double baseAngularStdDev = 0.06;
    public static final double megatag2Factor = 0.5;

}