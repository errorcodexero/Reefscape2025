package frc.robot.commands.gps;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.WaitForCoralCmd;
import frc.robot.subsystems.manipulator.ManipulatorGotoCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CollectCoralCmd extends ParallelCommandGroup {

    private static final Angle ArmCollectAngle = Degrees.of(90.0) ;
    private static final Distance ElevatorCollectHeight = Meters.of(1.0) ;

    public CollectCoralCmd(ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("CollectCoralCmd") ;
        addCommands(
            new ManipulatorGotoCmd(m, ElevatorCollectHeight, ArmCollectAngle),
            new WaitForCoralCmd(g),
            new RumbleGamepadCmd(Milliseconds.of(500))) ;
    }
}
