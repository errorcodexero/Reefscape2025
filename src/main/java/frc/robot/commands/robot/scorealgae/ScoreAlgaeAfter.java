package frc.robot.commands.robot.scorealgae;

import org.xerosw.util.XeroSequence;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositAlgaeCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ScoreAlgaeAfter extends Command {
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;
    private BrainSubsystem brain_ ;
    private XeroSequence sequence_ ;

    public ScoreAlgaeAfter(BrainSubsystem b, ManipulatorSubsystem m, GrabberSubsystem g) {
        m_ = m ;
        g_ = g ;
        brain_ = b ;
    }

    @Override
    public void initialize() {
        sequence_ = new XeroSequence();
        Command parallel = new ParallelCommandGroup(
            new DepositAlgaeCmd(g_),
            new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kScoreAlgaeReef2, 
            ManipulatorConstants.Arm.Positions.kScoreAlgaeReef2, true)
        ) ;
        sequence_.addCommands(
            new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kScoreAlgaeReef, 
                            ManipulatorConstants.Arm.Positions.kScoreAlgaeReef, true),
            new DepositAlgaeCmd(g_),
            new WaitCommand(2.0),
            new SetHoldingCmd(brain_, GamePiece.NONE),
            new GoToCmd(m_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)) ;
        sequence_.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
