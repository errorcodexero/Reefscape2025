package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public class GrabberConstants {

    public static final int kDistanceSensorInput = 0 ;

    public class Grabber {

        public static final int kMotorCANID = 1;
        public static final double kGearRatio = 1.0;
        public static final boolean kInverted = true; 
        public static final double kHoldingVoltage = -3.0 ;
        public static final double kDepositVoltage = 3.0 ;

        public class PID {
            public static final double kP = 0.6; 
            public static final double kI = 0.0 ;
            public static final double kD = 0.0 ;
            public static final double kV = 0.14 ;
            public static final double kA = 0.0 ;
            public static final double kG = 0.0 ;
            public static final double kS = 0.36102 ;
        }

        public class MotionMagic {
            public static final double kMaxVelocity = 60.0 ;
            public static final double kMaxAcceleration = 300.0 ;
            public static final double kJerk = 0.0 ;
        }

        public class CoralSensor {
            public static final int kChannel = 0;
        }

        public class AlgaeSensor {
            public static final int kChannel = 1;
        }

        public class DepositCoral {
            public static final AngularVelocity l1velocity = RotationsPerSecond.of(40.0) ;
            public static final AngularVelocity velocity = RotationsPerSecond.of(60.0) ;
            public static final Time delay = Milliseconds.of(300.0) ;
            public static final Time l1delay = Milliseconds.of(1000.0) ;
        }

        public class CollectCoral {
            public static final AngularVelocity kVelocity = RotationsPerSecond.of(15.0) ;
            public static final AngularVelocity kBackupVelocity = RotationsPerSecond.of(-20.0) ;
        }

        public class CollectAlgae {
            public static final AngularVelocity velocity = RotationsPerSecond.of(-20.0) ;
        }

        public class DepositAlgae {
            public static final AngularVelocity velocity = RotationsPerSecond.of(-120.0) ;
            public static final Time delay = Milliseconds.of(500.0) ;
        }

        public class Positions {
            public static final double CoralPositionVelocity = 0.0;

            public static final double ejectCoralVelocty = 1.0;
            public static final double ejectCoralWait = 0.0;

            public static final double collectAlgaeVelocity = 0.0;
            public static final double ejectAlgaeVelocity = 0.0;
            public static final double ejectAlgaeWait = 0.0;
        }

    }
}
