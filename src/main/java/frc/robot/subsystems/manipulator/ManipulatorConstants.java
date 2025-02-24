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
import edu.wpi.first.units.measure.Voltage;

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
        public static final AngularVelocity kVelTolerance = DegreesPerSecond.of(2);

        public static final Time kCurrentLimitTime = Seconds.of(1); 

        // The minimum and maximum arm angle, used to set the limits of travel
        public static final Angle kMaxArmAngle = Degrees.of(200.0);
        public static final Angle kMinArmAngle = Degrees.of(-5) ;

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
            public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(56.0) ;
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
            public static final double kEncoderCalibrationValue = 0.510;

            public static final int kAbsEncoder = 11; 
        }

        public class Positions {
            public static final Angle kStow = Degrees.of(20);
            public static final Angle kPlaceL1 = Degrees.of(0); 
            public static final Angle kPlaceL2 = Degrees.of(35); 
            public static final Angle kPlaceL3 = Degrees.of(35); 
            public static final Angle kPlaceL4 = Degrees.of(80);
            public static final Angle kKickbackAngle = Degrees.of(19); 
            public static final Angle kCollect = Degrees.of(13) ;
            public static final Angle kAlgaeReefCollectL2 = Degrees.of(190.0) ;
            public static final Angle kAlgaeReefCollectL3 = Degrees.of(190.0) ;
            public static final Angle kAlgaeReefHold = Degrees.of(200) ;
            public static final Angle kScoreAlgaeReef = Degrees.of(200) ;
            public static final Angle kScoreAlgaeReef2 = Degrees.of(200) ;
            public static final Angle kRaiseAngle = Degrees.of(19.0) ;
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

        public static final Current kCurrentLimit = Amps.of(60); 

        public static final Distance kPosTolerance = Centimeter.of(0.5) ;
        public static final LinearVelocity kVelTolerance = MetersPerSecond.of(0.01) ;

        public static final Time kCurrentLimitTime = Seconds.of(1); 

        public static final Voltage kCalibrateVoltage = Volts.of(-3) ;
        public static final int kCalibrateLoops = 8 ;
        public static final int kCalibrateLoopsFinal = kCalibrateLoops + 1 ;

        // Gear ratio between the motor and the wheel that the cable wraps around       
        // Used for simulation
        public static final double kGearRatio = 36.0 ;

        // The maximum height of the elevator
        public static final Distance kMaxHeight = Centimeters.of(177.0) ;

        // The minimum height of the elevator
        public static final Distance kMinHeight = Centimeters.of(0.0) ;

        // The MOI of the elevator, used only for simulation
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.1) ;

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
            public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(90) ;
            public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(325) ;
            public static final double kJerk = 0.0 ;
        }

        public class Positions {
            public static final Distance kStow = Centimeters.of(2);

            public static final Distance kPlaceL1 = Centimeters.of(0); 
            public static final Distance kPlaceL2 = Centimeters.of(25); 
            public static final Distance kPlaceL3 = Centimeters.of(65); 
            public static final Distance kPlaceL4 = Centimeters.of(131
            ); 
            public static final Distance kCollect = Centimeters.of(0.0) ;
            public static final Distance kAlgaeReefCollectL3 = Centimeters.of(92.0) ;
            public static final Distance kAlgaeReefCollectL2 = Centimeters.of(52.0) ;
            public static final Distance kAlgaeReefHold = Centimeters.of(12.0) ;
            public static final Distance kScoreAlgaeReef = Centimeters.of(12.0) ;
            public static final Distance kScoreAlgaeReef2 = Centimeters.of(12.0) ;
        }
    }
}
