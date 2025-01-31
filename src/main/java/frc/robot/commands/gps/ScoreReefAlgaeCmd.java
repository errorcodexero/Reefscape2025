package frc.robot.commands.gps;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.subsystems.grabber.DepositAlgaeCmd;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorGotoCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ScoreReefAlgaeCmd extends SequentialCommandGroup {
    private static final Angle ArmScoreAngle = Degrees.of(90.0) ;
    private static final Distance ElevatorScoreHeight = Meters.of(1.0) ;

    public ScoreReefAlgaeCmd(ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("ScoreReefAlgaeCmd") ;
        addCommands(
            new ManipulatorGotoCmd(m, ElevatorScoreHeight, ArmScoreAngle),
            new DepositAlgaeCmd(g),
            new SetHoldingCmd(RobotContainer.GamePiece.NONE),
            new RumbleGamepadCmd(Milliseconds.of(500))) ;
    }
}
