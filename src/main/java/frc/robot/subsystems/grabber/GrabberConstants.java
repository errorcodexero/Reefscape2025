package frc.robot.subsystems.grabber;

public class GrabberConstants {

    public class Grabber {

        public static final int kMotorCANID = 1;
        public static final double kGearRatio = 0.0;
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

        public class CoralFrontSensor {
            public static final int kChannel = 1;
            public static final boolean kInverted = false;
        }

        public class CoralBackSensor {
            public static final int kChannel = 2;
            public static final boolean kInverted = false;
        }

        public class CoralFunnelSensor {
            public static final int kChannel = 3;
            public static final boolean kInverted = false;
        }

        public class AlgaeUpperSensor {
            public static final int kChannel = 4;
            public static final boolean kInverted = false;
        }

        public class AlgaeLowerSensor {
            public static final int kChannel = 5;
            public static final boolean kInverted = false;
        }

        public class Positions {
            public static final double waitForCoralVelocity = 0.0;
            public static final double ejectCoralVelocty = 0.0;

            public static final double ejectCoralWait = 1.0;
        }

    }
}
