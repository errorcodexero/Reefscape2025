package frc.robot.commands.tests;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem ;

public class ManipulatorGrabAlgaeReefCmd extends SequentialCommandGroup {
    public final static Angle armAngle = Radians.of(-2.73) ;
    public final static Distance elevHeight = Centimeters.of(155.0) ;

    public ManipulatorGrabAlgaeReefCmd(ManipulatorSubsystem m, GrabberSubsystem g) {
        // addCommands(
        //     new GoToCmd(m, elevHeight, Degrees.of(0.0)),
        //     new GoToCmd(m, elevHeight, armAngle),
        //     new SetGrabberVelocityCmd(g, Volts.of(-4.0))) ;
    }
}
