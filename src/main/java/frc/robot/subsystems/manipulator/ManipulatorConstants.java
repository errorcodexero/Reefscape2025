package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ManipulatorConstants {

    public class Goto {
        public static final Angle kArmKeepOutMinAngle = Degrees.of(0.0) ;
        public static final Angle kArmKeepOutMaxAngle = Degrees.of(90.0) ;
        public static final Distance kElevatorKeepOutHeight = Meters.of(1.0) ;

        public static final Angle kArmTolerance = Degrees.of(5.0) ;
        public static final Distance kElevatorTolerance = Centimeters.of(20.0) ;
    }

    public class Arm {
       
        // motor CAN ID
        public static final int kMotorCANID = 0; 

        // gear ratio- degrees per rev
        public static final double kGearRatio = 9; 

        // if motor is inverted 
        public static final boolean kInverted = false; 

        // Moment of intertia for the arm, used only for simulation
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.001) ;

        public class PID {
            public static final double kP = 3 ; 
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.059557 ;
            public static final double kA = 0.087858 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        }

        public class MotionMagic {
            public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(0.5) ;
            public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(4.0) ;
            public static final PerUnit<AngularAccelerationUnit, TimeUnit> AngularJerk = RotationsPerSecondPerSecond.per(Seconds) ;
            public static final double kJerk = 12.0 ;
        }
    }

    public class Elevator{
        // motor CAN IDs
        public static final int kMotorCANID1 = 1; 
        public static final int kMotorCANID2 = 2; 

        // if motor is inverted 
        public static final boolean kInverted = false; 

        // Mapping from motor turns to linear height
        public static final double kMotorRevsToHeightMeters = 0.319185814 ;

        // Gear ratio between the motor and the wheel that the cable wraps around
        public static final double kGearRatio = 3 ;

        // The carriage mass
        public static final Mass kCarriageMass = Kilograms.of(1.0) ;

        // The drum radius in meters
        public static final Distance kDrumRadius = Inches.of(3.0) ;

        // The maximum height of the elevator
        public static final Distance kMaxHeight = Meters.of(2.0) ;

        // The minimum height of the elevator
        public static final Distance kMinHeight = Meters.of(0.0) ;

        public class PID {
            public static final double kP = 2000.431; 
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 7.4589 ;
            public static final double kA = 3.4787 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        }

        public class MotionMagic {
            public static final LinearVelocity kMaxVelocity = MetersPerSecond.of(32.0) ;
            public static final LinearAcceleration kMaxAcceleration = MetersPerSecondPerSecond.of(144.0) ;
            public static final double kJerk = 100000.0 ;
        }
    } 
}
