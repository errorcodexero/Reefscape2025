package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;

public class GrabberConstants {
    
    public class Grabber {
        // motor CAN ID
        public static final int kMotorCANID = 9; 

        // gear ratio- degrees per rev
        public static final double kGearRatio = 8; 

        // if motor is inverted 
        public static final boolean kInverted = true; 

        // Current limit for the grabber motor
        public static final Current kCurrentLimit = Amps.of(40.0) ;

        public static final Time kCurrentLimitTime = Seconds.of(1.0) ;

        // Tolerance for the grabber position
        public static final Angle kTolerance = Degrees.of(5.0) ;

        // Moment of inertia for the grabber
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0005) ;

        public class Velocity {
            // public class PID {
            //     public static final double kP = 0.75; 
            //     public static final double kI = 0.0 ;
            //     public static final double kD = 0.0 ;
            //     public static final double kV = 0.14 ;
            //     public static final double kA = 0.0 ;
            //     public static final double kG = 0.0 ;
            //     public static final double kS = 0.36102 ;
            // }

            public class PID {
                public static final double kP = 0.0;
                public static final double kI = 0.0 ;
                public static final double kD = 0.0 ;
                public static final double kV = 0.005 ;
                public static final double kA = 0.0 ;
                public static final double kG = 0.0 ;
                public static final double kS = 0.0 ;
            }
        }

        public class Position {
            public class PID {
                public static final double kP = 0.3;
                public static final double kI = 0.0 ;
                public static final double kD = 0.0 ;
                public static final double kV = 0.01 ;
                public static final double kA = 0.0 ;
                public static final double kG = 0.0 ;
                public static final double kS = 0.0 ;
            }        
        }
        
        public class MotionMagic {
            public static final double kMaxVelocity = 100.0 ;
            public static final double kMaxAcceleration = 100.0 ;
            public static final double kJerk = 0.0 ;
        }   
    } 

    public class Sensor {
        public static final int kCoralLow = 1 ;
        public static final int kCoralHigh = 2;
        public static final int kCoralFunnel = 3 ;
        public static final int kAlgaeHigh = 4 ;
        public static final int kAlgaeLow = 5 ;
    }

    public class Collect {
        public static final AngularVelocity kVelocity = RevolutionsPerSecond.of(20.0) ;
        public static final Time kDelay = Milliseconds.of(0) ;

        // Backup amount
        public static final Angle kBackup = Degrees.of(-10.0) ;
    }

    public class Place {
        public static final AngularVelocity kVelocity = RevolutionsPerSecond.of(10.0) ;
        public static final Time kDelay = Milliseconds.of(2000) ;
    }
}
