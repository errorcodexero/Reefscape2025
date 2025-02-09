package frc.robot.commands.robot.scoreground;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.commands.robot.SetHoldingCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositAlgaeCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ScoreGroundAlgaeCmd extends SequentialCommandGroup {
    private static final Angle ArmScoreAngle = Degrees.of(90.0) ;
    private static final Distance ElevatorScoreHeight = Meters.of(1.0) ;

    public ScoreGroundAlgaeCmd(BrainSubsystem b, ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("ScoreReefAlgaeCmd") ;
        addCommands(
            new GoToCmd(m, ElevatorScoreHeight, ArmScoreAngle),
            new DepositAlgaeCmd(g),
            new SetHoldingCmd(b, RobotContainer.GamePiece.NONE),
            new RumbleGamepadCmd(Milliseconds.of(500))) ;
    }
}
