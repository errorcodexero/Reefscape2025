package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class GrabberConstants {
    
    public class Grabber {
        // motor CAN ID
        public static final int kMotorCANID = 9; 

        // gear ratio- degrees per rev
        public static final double kGearRatio = 1; 

        // if motor is inverted 
        public static final boolean kInverted = true; 

        // Current limit for the grabber motor
        public static final Current kCurrentLimit = Amps.of(40.0) ;

        public class PID {
            public static final double kP = 0.75; 
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.14 ;
            public static final double kA = 0.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.36102 ;
        }

        public class MotionMagic {
            public static final double kMaxVelocity = 100.0 ;
            public static final double kMaxAcceleration = 100.0 ;
            public static final double kJerk = 0.0 ;
        }
    }   

    public class Sensor {
        public static final int kCoralLow = 3 ;
        public static final int kCoralHigh = 4;
        public static final int kCoralFunnel = 5 ;
        public static final int kAlgaeHigh = 2 ;
        public static final int kAlgaeLow = 6 ;
    }

    public class Collect {
        public static final AngularVelocity kVelocity = RevolutionsPerSecond.of(20.0) ;
        public static final Time kDelay = Milliseconds.of(0) ;
    }

    public class Place {
        public static final AngularVelocity kVelocity = RevolutionsPerSecond.of(10.0) ;
        public static final Time kDelay = Milliseconds.of(2000) ;
    }
}
