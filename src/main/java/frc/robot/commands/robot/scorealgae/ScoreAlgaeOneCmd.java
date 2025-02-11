package frc.robot.commands.robot.scorealgae;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ScoreAlgaeOneCmd extends Command {

    private XeroSequence sequence_ ;
    private BrainSubsystem b_ ;
    private ManipulatorSubsystem m_ ;

    public ScoreAlgaeOneCmd(BrainSubsystem b, ManipulatorSubsystem m) {
        setName("ScoreAlgaeCmd") ;

        b_ = b ;
        m_ = m ;
    }

    @Override
    public void initialize() {
        sequence_ = new XeroSequence() ;

        if (b_.gp() == GamePiece.ALGAE_LOW) {
            sequence_.addCommands(
                new GoToCmd(m_, ScoreAlgaeConstants.kAlgaeLowScoreElevatorHeight,
                                ScoreAlgaeConstants.kAlgaeLowScoreArmAngle)) ;
        }
        else if (b_.gp() == GamePiece.ALGAE_HIGH) {
            sequence_.addCommands(
                new GoToCmd(m_, ScoreAlgaeConstants.kAlgaeHighScoreElevatorHeight,
                                ScoreAlgaeConstants.kAlgaeHighScoreArmAngle)) ;
        }
    }
}
