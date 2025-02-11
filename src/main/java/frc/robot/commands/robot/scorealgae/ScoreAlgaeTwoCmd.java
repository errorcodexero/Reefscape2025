package frc.robot.commands.robot.scorealgae;

import static edu.wpi.first.units.Units.Milliseconds;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.commands.robot.SetHoldingCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositAlgaeCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ScoreAlgaeTwoCmd extends Command {
    private XeroSequence sequence_ ;
    private BrainSubsystem b_ ;
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;

    public ScoreAlgaeTwoCmd(BrainSubsystem b, ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("ScoreAlgaeCmd") ;

        b_ = b ;
        m_ = m ;
        g_ = g ;
    }

    @Override
    public void initialize() {
        sequence_ = new XeroSequence() ;

        if (b_.gp() == GamePiece.ALGAE_LOW) {
            sequence_.addCommands(
                new GoToCmd(m_, ScoreAlgaeConstants.kAlgaeLowScoreElevatorHeight,
                                ScoreAlgaeConstants.kAlgaeLowScoreArmAngle),
                new DepositAlgaeCmd(g_),
                new SetHoldingCmd(b_, GamePiece.NONE),
                new RumbleGamepadCmd(Milliseconds.of(500))) ;
        }
        else if (b_.gp() == GamePiece.ALGAE_HIGH) {
            sequence_.addCommands(
                new GoToCmd(m_, ScoreAlgaeConstants.kAlgaeHighScoreElevatorHeight,
                                ScoreAlgaeConstants.kAlgaeHighScoreArmAngle),
                new DepositAlgaeCmd(g_),
                new SetHoldingCmd(b_, GamePiece.NONE),
                new RumbleGamepadCmd(Milliseconds.of(500))) ;
        }
    }
}
