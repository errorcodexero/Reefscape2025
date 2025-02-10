package frc.robot.commands.robot.collectground;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class CollectGroundConstants {
    public static final Distance kElevatorImmdHeight = Centimeters.of(6.0) ;

    public static final Distance kElevatorStowHeight = Centimeters.of(1.0) ;
    public static final Angle kArmStowAngle = Degrees.of(0.0) ;

    public static final Distance kElevatorCollectHeight = Centimeters.of(8.0) ;
    public static final Angle kArmCollectAngle = Degrees.of(-75.0) ;

    public static final Distance kElevatorStoreHeight = Centimeters.of(1.0) ;
    public static final Angle kArmStoreAngle = Degrees.of(-75.0) ;
}
