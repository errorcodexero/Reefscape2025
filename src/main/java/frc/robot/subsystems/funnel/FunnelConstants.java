package frc.robot.subsystems.funnel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class FunnelConstants {
    
    public static final int funnelCanId = 6;
    public static final int funnelSensorId = 10 ;
    
    public static final Current currentLimit = Amps.of(40);
    public static final Time lowerTime = Seconds.of(1);

    public static final Angle kClimbPosition = Rotations.of(0.81) ;
    public static final Angle kNormalPosition = Rotations.of(0.0) ;
    public static final Angle kMaxPosition = Rotations.of(0.85) ;
    public static final Angle kMinPosition = Rotations.of(-0.05) ;
    public static final Angle kTolerence = Rotations.of(0.1) ;

    public static final AngularVelocity maxVelocity = RotationsPerSecond.of(40);
    public static final AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(40);
    public static final double jerk = 0;

    public static class MotorPids {
        public static final double kP = 8;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.1;
        public static final double kA = 0;
        public static final double kG = 0;
        public static final double kS = 0;
    }
}
