package frc.robot.commands.robot.placecoral;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class PlaceCoralConstants {
    public static final LinearVelocity DriveToMaxVelocity = MetersPerSecond.of(1.0) ;
    public static final LinearAcceleration DriveToMaxAcceleration = MetersPerSecondPerSecond.of(1.0) ;
    public static final AngularVelocity DriveToMaxAngularVelocity = DegreesPerSecond.of(60.0) ;
    public static final AngularAcceleration DriveToMaxAngularAcceleration = DegreesPerSecondPerSecond.of(60.0) ;

    public static final LinearVelocity BackupMaxVelocity = MetersPerSecond.of(1.0) ;
    public static final LinearAcceleration BackupMaxAcceleration = MetersPerSecondPerSecond.of(1.0) ;
    public static final AngularVelocity BackupMaxAngularVelocity = DegreesPerSecond.of(60.0) ;
    public static final AngularAcceleration BackupMaxAngularAcceleration = DegreesPerSecondPerSecond.of(60.0) ;

    public static class Place {
        public static final Angle ArmAngle[] = {
            Degrees.of(90.0),
            Degrees.of(90.0),
            Degrees.of(90.0),
            Degrees.of(90.0),
        } ;

        protected static final Distance ElevatorHeight[] = {
            Meters.of(1.0),
            Meters.of(1.0),
            Meters.of(1.0),
            Meters.of(1.0)
        } ;
    }
}
