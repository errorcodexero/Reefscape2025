package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class ClimberConstants {
    // this information would go inside classes for each motor in the subsystem

    public static final int kAttachedSensorOne = 8;
    public static final int kAttachedSensorTwo = 9;

    public class Climber{
       
        //climber motor CAN ID
        public static final int kMotorCANID = 5; 

        // gear ratio
        public static final double kGearRatio = 266.67; 

        // if motor is inverted 
        public static final boolean kInverted = false; 
        public static final Current kcurrentLimit = Amps.of(40);
        public static final Time kCurrentLimitTime = Seconds.of(1); 
        public static final Angle kPosTolerance = null;

        public static final AngularVelocity kVelTolerance = DegreesPerSecond.of(0);
        
        //TODO: Add the correct values
        public static final Angle kMaxClimberAngle = Degrees.of(0);
        public static final Angle kMinClimberAngle = Degrees.of(0) ;

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

        public class Position {
            //TODO : Add the correct angles
            public static final Angle kStowed = Degrees.of(0);
            public static final Angle kPrepped = Degrees.of(0);
            public static final Angle kClimbed = Degrees.of(0);
        }
    }   
}
