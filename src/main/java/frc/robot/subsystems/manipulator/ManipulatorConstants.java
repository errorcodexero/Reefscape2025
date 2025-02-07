package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class ManipulatorConstants {

    public class Arm {
       
        // motor CAN ID
        public static final int kMotorCANID = 2; 

        // if motor is inverted 
        public static final boolean kInverted = false; 

        public static final double kGearRatio = 0; 

        public static final String kCANBusName = null; 

        public static final Current kCurrentLimit = Amps.of(40); 

        public static final Angle kPosTolerance = Degrees.of(1);

        public static final Time kCurrentLimitTime = Seconds.of(1); 

        public class PID {
            public static final double kP = 0.0; 
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
            public static final double kG = 0.0;
            public static final double kS = 0.0;
        }

        public class MotionMagic {
            public static final double kMaxVelocity = 0.0;
            public static final double kMaxAcceleration = 0.0;
            public static final double kJerk = 0.0;
        }

        public class ThruBoreEncoder {
            // the encoder mapper needs double values, so these constants don't use the Units library
            // robot min and max are in degrees
            public static final double kRobotMax = 360; 
            public static final double kRobotMin = 0; 
            public static final double kEncoderMax = 1; 
            public static final double kEncoderMin = 0; 
            public static final double kRobotCalibrationValue = 0; 
            public static final double kEncoderCalibrationValue = 0;

            public static final int kEncoderSource = 0; 
        }
    }

    public class Elevator {

        // motor CAN ID
        public static final int kMotorCANID = 3;
        public static final int kMotorCANID2 = 4;

        // if motor is inverted 
        public static final boolean kInverted = false;

        // meters that elevator moves per revolution of motor
        public static final double kMetersPerRev = 0; 

        public static final String kCANBusName = null; 

        public static final Current kCurrentLimit = Amps.of(40); 

        public static final Distance kPosTolerance = Meters.of(0.01); 

        public static final Time kCurrentLimitTime = Seconds.of(1); 

        public class PID {
            public static final double kP = 0.0; 
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
            public static final double kG = 0.0;
            public static final double kS = 0.0;
        }

        public class MotionMagic {
            public static final double kMaxVelocity = 0.0;
            public static final double kMaxAcceleration = 0.0;
            public static final double kJerk = 0.0;
        }
    }

    public class Keepout {
        public static final Distance kKeepoutHeight = null; 
        public static final Angle kKeepoutMinAngle = null;
        public static final Angle kKeepoutMaxAngle = null;
    }
}
