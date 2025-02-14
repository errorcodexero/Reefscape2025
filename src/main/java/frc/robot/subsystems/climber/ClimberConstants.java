package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class ClimberConstants {
    // this information would go inside classes for each motor in the subsystem

    public class Climber{
       
        //climber motor CAN ID
        public static final int kMotorCANID = 5; 

        // gear ratio
        public static final double kGearRatio = 307.20; 

        // if motor is inverted 
        public static final boolean kInverted = false; 

        public static final Angle kPosTolerance = null;
        

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
