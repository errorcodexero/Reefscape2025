// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;

/**
* This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
* on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
* (log replay from a file).
*/
public final class Constants {
    
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    
    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        
        /** Running a physics simulator. */
        SIM,
        
        /** Replaying from a log file. */
        REPLAY
    }
    
    public static class DriveConstants {
        
        public static final double slowModeJoystickMultiplier = 0.4;
        
    }
    
    public static class FieldConstants {
        
        public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        
    }

    public static class VisionConstants {

        // Limelight Names
        public static final String frontLimelightName = "frontlimelight";
        public static final String backLimelightName = "backlimelight";

        // Odometry Filtering Configuration
        
        public static final int minimumTagCount = 1;
        public static final double maximumAmbiguity = 0.3; // For Photonvision Sim

        // Standard Deviation Factors, (we need to talk about how stddev is calculated, dont change until then)
        public static final double baseLinearStdDev = 0.02;
        public static final double baseAngularStdDev = 0.06;
        public static final double megatag2Factor = 0.5;

    }
    
}
