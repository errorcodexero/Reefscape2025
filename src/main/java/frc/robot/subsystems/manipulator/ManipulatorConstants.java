package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ManipulatorConstants {

    public class Goto {
        public static final Angle kArmKeepOutMinAngle = Degrees.of(0.0) ;
        public static final Angle kArmKeepOutMaxAngle = Degrees.of(90.0) ;
        public static final Distance kElevatorKeepOutHeight = Meters.of(1.0) ;

        public static final Angle kArmTolerance = Degrees.of(5.0) ;
        public static final Distance kElevatorTolerance = Centimeters.of(20.0) ;
    }

    public class Positions {
        public static final Angle kStowedAngle = Degrees.of(0.0) ;
        public static final Distance kStowedHeight = Centimeters.of(59.0) ;
    }

    public class Arm {
       
        // motor CAN ID
        public static final int kMotorCANID = 8; 

        public static final Angle kMaxArmAngle = Degrees.of(90.0) ;
        public static final Angle kMinArmAngle = Degrees.of(-180.0) ;

        // gear ratio- degrees per rev
        public static final double kGearRatio = 36; 

        // if motor is inverted 
        public static final boolean kInverted = true; 

        // Current limit
        public static final Current kCurrentLimit = Amps.of(40.0) ;

        // Moment of intertia for the arm, used only for simulation
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.001) ;

        // The starting angle of the ARM as seen by the absolute encoder
        public static final Angle kStartAbsEncoderAngle = Degrees.of(-45.0) ;

        public class PID {
            public static final double kP = 24.0 ; 
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.45 ;
            public static final double kA = 0.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        }

        public class MotionMagic {
            public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(32.0) ;
            public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(1000.0) ;
            public static final double kJerk = 8000.0 ;
        }

        public class ThruBoreEncoder {
            public static final int kDutyCyclePin = 0 ;
            public static final double kRobotMax = 180.0 ;
            public static final double kRobotMin = -180.0 ;
            public static final double kEncoderMax = 1.0 ;
            public static final double kEncoderMin = 0.0 ;
            public static final double kRobotCalibrationValue = 0.0 ;
            public static final double kEncoderCalibrationValue = 0.0 ;
        }
    }

    public class Elevator{
        // motor CAN IDs
        public static final int kMotorCANID1 = 4; 
        public static final int kMotorCANID2 = 5; 

        // if motor is inverted 
        public static final boolean kInverted = false; 

        // Mapping from motor turns to linear height
        public static final double kMotorRevsPerMeters = 42.375 ;

        // Current limit
        public static final Current kCurrentLimit = Amps.of(40.0) ;

        // Gear ratio between the motor and the wheel that the cable wraps around       
        public static final double kGearRatio = 36.0 ;

        // The maximum height of the elevator
        public static final Distance kMaxHeight = Centimeters.of(177.0) ;

        // The minimum height of the elevator
        public static final Distance kMinHeight = Centimeters.of(59.0) ;

        // The MOI of the elevator, used only for simulation
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.004) ;

        public class PID {
            public static final double kP = 8.0; 
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.15 ;
            public static final double kA = 0.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.5 ;
        }

        public class MotionMagic {
            public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(32) ;
            // public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(16) ;
            public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(1000) ;
            public static final double kJerk = 8000.0 ;
        }
    } 
}
