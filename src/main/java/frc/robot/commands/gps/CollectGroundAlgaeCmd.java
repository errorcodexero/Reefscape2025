package frc.robot.commands.gps;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.WaitForAlgaeCmd;
import frc.robot.subsystems.manipulator.ManipulatorGotoCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CollectGroundAlgaeCmd extends SequentialCommandGroup {
    private static final Angle ArmPlaceAngle = Degrees.of(90.0) ;
    private static final Distance ElevatorPlaceHeight = Meters.of(1.0) ;

    private static final Angle ArmStowAngle = Degrees.of(90.0) ;
    private static final Distance ElevatorStowHeight = Meters.of(1.0) ;

    public CollectGroundAlgaeCmd(ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("PlaceCoralCmd") ;

            addCommands(
                new ManipulatorGotoCmd(m, ElevatorPlaceHeight, ArmPlaceAngle),
                new WaitForAlgaeCmd(g, false),
                new SetHoldingCmd(RobotContainer.GamePiece.ALGAE_LOW),
                new ManipulatorGotoCmd(m, ElevatorStowHeight, ArmStowAngle),
                new RumbleGamepadCmd(Milliseconds.of(500))) ;
    }
}
