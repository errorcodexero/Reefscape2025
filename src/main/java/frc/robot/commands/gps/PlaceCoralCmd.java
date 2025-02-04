package frc.robot.commands.gps;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PlaceCoralCmd extends SequentialCommandGroup {    

    protected static final Angle ArmPlaceAngle[] = {
        Degrees.of(90.0),
        Degrees.of(90.0),
        Degrees.of(90.0),
        Degrees.of(90.0),
    } ;

    protected static final Distance ElevatorPlaceHeight[] = {
        Meters.of(1.0),
        Meters.of(1.0),
        Meters.of(1.0),
        Meters.of(1.0)
    } ;
}
