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

    public static final int kAttachedSensor = 13;

    public static final double kMinAbsEncoderRollover = 0.3289;
    public static final double kMConvertAbsToRobot = 167.286 ;
    public static final double kBConvertAbsToRobot = -110.743 ;

    public class ThruBoreEncoder {
        // the encoder mapper needs double values, so these constants don't use the Units library
        // robot min and max are in degrees
        public static final double kRobotMax = 275 ;
        public static final double kRobotMin = 151 ; 
        public static final double kEncoderMax = 1; 
        public static final double kEncoderMin = 0; 
        public static final double kRobotCalibrationValue = 0;
        public static final double kEncoderCalibrationValue = 0.566;

        public static final int kAbsEncoder = 2; 
    }

    public class Climber{
       
        //climber motor CAN ID
        public static final int kMotorCANID = 5; 

        // gear ratio
        public static final double kGearRatio = 480.0 ;

        // if motor is inverted 
        public static final boolean kInverted = false; 
        public static final Current kcurrentLimit = Amps.of(40);
        public static final Time kCurrentLimitTime = Seconds.of(1); 
        public static final Angle kPosTolerance = Degrees.of(2.0) ;
        public static final AngularVelocity kVelTolerance = DegreesPerSecond.of(0.5) ;
        
        public static final Angle kMaxClimberAngle = Degrees.of(100);
        public static final Angle kMinClimberAngle = Degrees.of(-48) ;

        public class PID {
            public static final double kP = 2.0; 
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.02 ;
            public static final double kA = 0.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.0 ;
        }

        public class MotionMagic {
            public static final double kMaxVelocity = 160.0 ;
            public static final double kMaxAcceleration = 600.0 ;
            public static final double kJerk = 0.0 ;
        }
        
        // Climb voltage
        public static final double kClimbVoltage = -5.0 ;

        public class Position {
            public static final Angle kStowed = Degrees.of(0.0);
            public static final Angle kPrepped = Degrees.of(88.5
            );
            public static final Angle kClimbed = Degrees.of(-30.0);
            public static final Angle kReapplyThreshold = Degrees.of(-26.0);
        }
    }   
}
