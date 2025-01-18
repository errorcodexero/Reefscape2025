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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
* This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
* on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
* (log replay from a file).
*/
public final class Constants {

    /**
     * CONFIGURATION
     */
    
    // Sets the currently running robot.
    private static final RobotType robotType = RobotType.SIMBOT;

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

    /**
     * ROBOT STATE
     */
    
    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        
        /** Running a physics simulator. */
        SIM,
        
        /** Replaying from a log file. */
        REPLAY
    }

    public static enum RobotType {
        /** The Alpha Bot (aka the 2024 practice bot drivebase) */
        ALPHA,

        /** The Competition Bot */
        COMPETITION,

        /** The Practice Bot (aka the old allegro) */
        PRACTICE,

        /** The Sim Bot */
        SIMBOT
    }

    // This is only a fallback! This will not change the robot type.
    private static final RobotType defaultRobotType = RobotType.ALPHA;

    private static final Alert invalidRobotType = new Alert(
        "Invalid RobotType selected. Defaulting to " + defaultRobotType.toString(),
        AlertType.kWarning
    );

    public static RobotType getRobot() {
        if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
            invalidRobotType.set(true);
            return defaultRobotType;
        }

        return robotType;
    }

    public static final Mode getMode() {
        return switch(getRobot()) {
            case SIMBOT -> Mode.SIM;
            default -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
        };
    }
    
}
