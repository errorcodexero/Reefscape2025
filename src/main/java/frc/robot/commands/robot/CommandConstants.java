package frc.robot.commands.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class CommandConstants {
    public static class ReefDrive {
        public static final LinearVelocity kMaxDriveVelocity = MetersPerSecond.of(3.0) ;
        public static final LinearAcceleration kMaxDriveAcceleration = MetersPerSecondPerSecond.of(3.0) ;
    }
}
