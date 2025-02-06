package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ClimberConstants {
    // this information would go inside classes for each motor in the subsystem

    public static Angle kPositionTolerance = Degrees.of(5.0) ;
    public static AngularVelocity kVelocityTolerance = DegreesPerSecond.of(5.0) ;

    public class ClimberArm {
       
        // motor CAN ID
        public static final int kMotorCANID = 1; 

        // gear ratio- degrees per rev
        public static final double kGearRatio = 32; 

        // if motor is inverted 
        public static final boolean kInverted = false; 

        // Current limit
        public static final Current kCurrentLimit = Amps.of(80.0) ;

        // Moment of inertia for the grabber
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0005) ;             

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

    public class Sensors {
        public static final int kCageSensor1 = 10 ;
        public static final int kCageSensor2 = 11 ;
        public static final int kCageSensor3 = 12 ;
        public static final int kDoorSensor1 = 13 ;
        public static final int kDoorSensor2 = 14 ;
    }

    public class Positions {
        public static final Angle kDeployed = Degrees.of(0.0) ;
        public static final Angle kRetracted = Degrees.of(90.0) ;
        public static final Angle kClimbPosition = Degrees.of(180.0) ;
    }
}