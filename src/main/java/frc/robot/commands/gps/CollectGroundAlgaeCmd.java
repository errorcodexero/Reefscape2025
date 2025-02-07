package frc.robot.commands.gps;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.CollectAlgaeCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CollectGroundAlgaeCmd extends SequentialCommandGroup {
    private static final Distance ElevatorCollectHeight = Meters.of(1.0) ;
    private static final Angle ArmAngleAngle = Degrees.of(90.0) ;
    private static final Distance ElevatorStowHeight = Meters.of(0.0) ;
    private static final Angle ArmStowAngle = Degrees.of(0.0) ;

    public CollectGroundAlgaeCmd(BrainSubsystem b, ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("PlaceCoralCmd") ;

        addCommands(
            new GoToCmd(m, ElevatorCollectHeight, ArmAngleAngle),
            new CollectAlgaeCmd(g),
            new SetHoldingCmd(b, RobotContainer.GamePiece.ALGAE_LOW),
            new GoToCmd(m, ElevatorStowHeight, ArmStowAngle),
            new RumbleGamepadCmd(Milliseconds.of(500))) ;
    }
}
