package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ManipulatorConstants {

    public class Goto {
        public static final Angle kArmKeepOutMinAngle = Degrees.of(0.0) ;
        public static final Angle kArmKeepOutMaxAngle = Degrees.of(90.0) ;
        public static final Distance kElevatorKeepOutHeight = Meters.of(1.0) ;

        public static final Angle kArmTolerance = Degrees.of(1.0) ;
        public static final Distance kElevatorTolerance = Centimeters.of(5.0) ;
    }

    public class Arm {
       
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

        // if motor is inverted 
        public static final boolean kInverted = false; 

        // Mapping from motor turns to linear height
        public static final double kMotorRevsToHeightMeters = 0.1 ;

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
