package frc.robot.subsystems.funnel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
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
    
    public class ThruBoreEncoder {
        // the encoder mapper needs double values, so these constants don't use the Units library
        // robot min and max are in degrees
        public static final double kRobotMax = 180; 
        public static final double kRobotMin = -180; 
        public static final double kEncoderMax = 1; 
        public static final double kEncoderMin = 0; 
        public static final double kRobotCalibrationValue = 0;
        public static final double kEncoderCalibrationValue = 0.539;

        public static final int kAbsEncoder = 3; 
    }

    public static final Current currentLimit = Amps.of(40);
    public static final Time lowerTime = Seconds.of(1);

    public static final AngularVelocity maxVelocity = RotationsPerSecond.of(40);
    public static final AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(40);
    public static final double jerk = 40;

    public static final double kGearRatio = 40.0 / 14.0;

    public static final Angle funnelArmMaxAngle = Degrees.of(0.0) ;
    public static final Angle funnelArmClimbAngle = Degrees.of(-87.0) ;
    public static final Angle funnelArmMinAngle = Degrees.of(-90.0) ;

    public static class MotorPids {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 0;
        public static final double kS = 0;
    }
    
}
