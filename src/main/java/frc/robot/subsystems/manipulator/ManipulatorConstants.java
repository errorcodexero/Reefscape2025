package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;

public class ManipulatorConstants {

    public class Arm {
       
        // motor CAN ID
        public static final int kMotorCANID = 2; 

        // if motor is inverted 
        public static final boolean kInverted = true; 

        public static final double kGearRatio = 36.0 ;

        public static final String kCANBusName = null; 

        public static final Current kCurrentLimit = Amps.of(40); 

        public static final Angle kPosTolerance = Degrees.of(1);
        public static final AngularVelocity kVelTolerance = DegreesPerSecond.of(1);

        public static final Time kCurrentLimitTime = Seconds.of(1); 

        // The minimum and maximum arm angle, used to set the limits of travel
        public static final Angle kMaxArmAngle = Degrees.of(179.0);
        public static final Angle kMinArmAngle = Degrees.of(-90) ;

        // Moment of intertia for the arm, used only for simulation
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.001) ;

        // The starting angle of the ARM as seen by the absolute encoder
        public static final Angle kStartAbsEncoderAngle = Degrees.of(-45.0) ;        

        public class PID {
            public static final double kP = 4.0 ; 
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.1 ;
            public static final double kA = 0.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        }

        public class MotionMagic {
            public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(32.0) ;
            public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(300.0) ;
            public static final double kJerk = 3000.0 ;
        }

        public class ThruBoreEncoder {
            // the encoder mapper needs double values, so these constants don't use the Units library
            // robot min and max are in degrees
            public static final double kRobotMax = 180; 
            public static final double kRobotMin = -180; 
            public static final double kEncoderMax = 0; 
            public static final double kEncoderMin = 1; 
            public static final double kRobotCalibrationValue = 0; 
            public static final double kEncoderCalibrationValue = 0.515;

            public static final int kEncoderSource = 3; 
        }
    }

    public class Elevator {

        // motor CAN ID
        public static final int kMotorFrontCANID = 3;
        public static final int kMotorBackCANID = 4;

        // if motor is inverted 
        public static final boolean kInverted = true;

        // meters that elevator moves per revolution of motor
        public static final double kMetersPerRev = 1.0 / 42.375  ;

        public static final String kCANBusName = "" ;

        public static final Current kCurrentLimit = Amps.of(40); 

        public static final Distance kPosTolerance = Meters.of(0.01) ;
        public static final LinearVelocity kVelTolerance = MetersPerSecond.of(0.1) ;

        public static final Time kCurrentLimitTime = Seconds.of(1); 

        // Gear ratio between the motor and the wheel that the cable wraps around       
        // Used for simulation
        public static final double kGearRatio = 36.0 ;

        // The maximum height of the elevator
        public static final Distance kMaxHeight = Centimeters.of(177.0) ;

        // The minimum height of the elevator
        public static final Distance kMinHeight = Centimeters.of(0.0) ;

        // The MOI of the elevator, used only for simulation
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.004) ;

        public class PID {
            public static final double kP = 20.0; 
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.15 ;
            public static final double kA = 0.0 ;
            public static final double kG = 0.288 ;
            public static final double kS = 0.0 ;
        }

        public class MotionMagic {
            public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(50) ;
            public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(300) ;
            public static final double kJerk = 0.0 ;
        }
    }

    public class Keepout {
        public static final Distance kKeepoutHeight = Meters.of(0.4); 
        public static final Angle kKeepoutMinAngle = Degrees.of(15.0) ;
        public static final Angle kKeepoutMaxAngle = Degrees.of(90.0) ;
    }

    public class Positions {
        public static final Distance kStowedHeight = Meters.of(0.0);
        public static final Angle kStowedAngle = Degrees.of(0.0);

        public static final Distance kCollectHeight = Meters.of(0.0);
        public static final Angle kCollectAngle = Degrees.of(0.0);

        public static final Distance kLowScoreHeight = Meters.of(0.0);
        public static final Angle kLowScoreAngle = Degrees.of(0.0);

        public static final Distance kHighScoreHeight = Meters.of(0.0);
        public static final Angle kHighScoreAngle = Degrees.of(0.0);

        public static final Distance kEjectAlgaeHeight = Meters.of(1.0);
        public static final Angle kEjectAlgaeAngle = Degrees.of(-30.0);
    }
}
