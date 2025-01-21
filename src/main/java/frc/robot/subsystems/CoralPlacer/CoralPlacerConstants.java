package frc.robot.subsystems.CoralPlacer;

public class CoralPlacerConstants {
    // this information would go inside classes for each motor in the subsystem

    public class Arm{
       
        // motor CAN ID
        public static final int kMotorCANID = 0; 

        // gear ratio- degrees per rev
        public static final double kGearRatio = 0; 

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

    public class Elevator{
        // motor CAN ID
        public static final int kMotorCANID = 0; 

        // gear ratio- degrees per rev
        public static final double kGearRatio = 0; 

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
}
