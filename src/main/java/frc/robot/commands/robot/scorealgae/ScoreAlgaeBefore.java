package frc.robot.commands.robot.scorealgae;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ScoreAlgaeBefore extends Command {
    private XeroSequence sequence_ ;

    public ScoreAlgaeBefore(ManipulatorSubsystem m_) {
        sequence_ = new XeroSequence();
        sequence_.addCommands(
            new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kScoreAlgaeReef, 
                            ManipulatorConstants.Arm.Positions.kScoreAlgaeReef, true)) ;

        sequence_.schedule();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            sequence_.cancel();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return sequence_.isComplete();
    }    
}
