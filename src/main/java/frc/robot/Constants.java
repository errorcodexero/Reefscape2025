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
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

/**
* This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
* on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
* (log replay from a file).
*/
public final class Constants {

    public static final boolean propogateExceptionOnSubsystemCreateFail = true ;

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

    public static class ReefConstants {
        /**
         * The maximum angle from the ROBOT to the nearest face of the REEF for it to be considered targeting that face.
         */
        public static final Angle maximumAngleToFace = Degrees.of(40);

        /**
         * The maximum distance from the ROBOT to the nearest face of the REEF for it to be considered targeting that face.
         */
        public static final Distance maximumDistanceToFace = Meters.of(3);

        /**
         * The distance from the center of the ROBOT to the TAG while placing CORAL.
         */
        public static final Distance distanceFromTagCoral = Inches.of(20);

        /**
         * The distance from the center of the ROBOT to the TAG while collecting ALGAE.
         */
        public static final Distance distanceFromTagAlgae = Inches.of(20);

        /**
         * The offset from the center of the TAG to where we want the ARM to be positioned.
         * (Half the distance between pipes on the REEF)
         */
        public static final Distance leftRightOffset = Inches.of(13 / 2);

        /**
         * How far to back up from the CORAL scoring pose.
         */
        public static final Distance backupDistanceCoral = Inches.of(28);

        /**
         * How far to back up from the ALGAE scoring pose.
         */
        public static final Distance backupDistanceAlgae = Inches.of(28);

        /**
         * The distance from the center of the ROBOT to the ARM.
         * (LEFT of the robot is POSITIVE)
         */
        public static final Distance robotToArm = Inches.of(0.125);
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

    public enum ReefLevel {
        L1(0),
        L2(1),
        L3(2),
        L4(3),
        AskBrain(4) ;

        private final int index;

        private ReefLevel(int index) {
            this.index = index ;
        }

        public int getindex() {
            return index;
        }
    }

    // This is only a fallback! This will not change the robot type.
    private static final RobotType defaultRobotType = RobotType.COMPETITION;

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
