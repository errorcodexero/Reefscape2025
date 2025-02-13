package frc.robot.commands.robot;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CollectAlgaeReefPt1Cmd extends Command {

    private XeroSequence sequence_; 
    private ManipulatorSubsystem manipulator_; 
    private BrainSubsystem brain_;
    private AlgaeLevel State_;
    private int algaeLevel_;
    private Distance elevatorHeight_;

    private enum AlgaeLevel {
        Waiting,
        L2,
        L3,
        Executing
    }

    // Need different elevator heights in constants for L2 and L3 collect

    public CollectAlgaeReefPt1Cmd(ManipulatorSubsystem manipulator, BrainSubsystem brain) {
        addRequirements(manipulator, brain); 

        manipulator_ = manipulator;
        brain_ = brain;
    }

    // COMMANDS NEEDED: 
    // GoToCmd

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        algaeLevel_ = brain_.level();
        State_ = AlgaeLevel.Waiting;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (State_) {
            case Waiting:
                if (algaeLevel_ == 2 && !(manipulator_.isElevAtTarget() && manipulator_.isArmAtTarget())) {
                    State_ = AlgaeLevel.L2;
                } 
                else if (algaeLevel_ == 3 && !(manipulator_.isElevAtTarget() && manipulator_.isArmAtTarget())) {
                    State_ = AlgaeLevel.L3;
                }
                break;
            case L2:
                elevatorHeight_ = ManipulatorConstants.Elevator.Positions.kReefCollect;
                    State_ = AlgaeLevel.Executing;
                break;
            case L3:
                elevatorHeight_ = ManipulatorConstants.Elevator.Positions.kReefCollectL3;
                    State_ = AlgaeLevel.Executing;
                break;

            case Executing:
                GoToCmd goToCmd = new GoToCmd(manipulator_, elevatorHeight_, ManipulatorConstants.Arm.Positions.kReefCollect);
                sequence_.addCommands(goToCmd);
                sequence_.schedule();
                State_ = AlgaeLevel.Waiting;
            }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        sequence_.cancel(); 
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return sequence_.isComplete();
    }
}
