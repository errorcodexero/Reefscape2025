package frc.robot.subsystems.funnel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class FunnelConstants {
    // this information would go inside classes for each motor in the subsystem

    public static Angle kPositionTolerance = Degrees.of(5.0) ;
    public static AngularVelocity kVelocityTolerance = DegreesPerSecond.of(5.0) ;

    public class Funnel {
       
        // motor CAN ID
        public static final int kMotorCANID = 2; 

        // gear ratio- degrees per rev
        public static final double kGearRatio = 0; 

        // if motor is inverted 
        public static final boolean kInverted = false; 

        // Current limit
        public static final Current kCurrentLimit = Amps.of(40.0) ;

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

    public class Positions {
        public static final Angle kDownPosition = Degrees.of(0.0) ;
        public static final Angle kUpPosition = Degrees.of(90.0) ;
    }
}