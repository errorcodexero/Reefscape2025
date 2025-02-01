package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public class GrabberConstants {
    
    public class Grabber {
        // motor CAN ID
        public static final int kMotorCANID = 3; 

        // gear ratio- degrees per rev
        public static final double kGearRatio = 1; 

        // if motor is inverted 
        public static final boolean kInverted = false; 

        public class PID {
            public static final double kP = 0.0; 
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.0 ;
            public static final double kA = 0.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        }

        public class MotionMagic {
            public static final double kMaxVelocity = 0.0 ;
            public static final double kMaxAcceleration = 0.0 ;
            public static final double kJerk = 0.0 ;
        }
    }   

    public class Sensor {
        public static final int kCoralLow = 1 ;
        public static final int kCoralHigh = 2 ;
        public static final int kCoralFunnel = 3 ;
        public static final int kAlgaeHigh = 4 ;
        public static final int kAlgaeLow = 5 ;
    }

    public class Collect {
        public static final AngularVelocity kVelocity = RevolutionsPerSecond.of(10.0) ;
        public static final Time kDelay = Milliseconds.of(250) ;
    }

    public class Place {
        public static final AngularVelocity kVelocity = RevolutionsPerSecond.of(10.0) ;
        public static final Time kDelay = Milliseconds.of(250) ;
    }
}
