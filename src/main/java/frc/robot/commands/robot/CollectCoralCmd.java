package frc.robot.commands.robot;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.WaitForCoralCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CollectCoralCmd extends Command {
    private XeroSequence sequence_;
    private ManipulatorSubsystem manipulator_;
    private GrabberSubsystem grabber_;
    private BrainSubsystem brain_;

    public CollectCoralCmd(BrainSubsystem brain, ManipulatorSubsystem manipulator, GrabberSubsystem grabber) {
        manipulator_ = manipulator;
        grabber_ = grabber;
        brain_ = brain;
    }

    // COMMANDS NEEDED:
    // GoToCmd
    // WaitForCoral

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        sequence_ = new XeroSequence();
        sequence_.addCommands(
            new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kCollect, ManipulatorConstants.Arm.Positions.kCollect),
            new WaitForCoralCmd(grabber_),
            new SetHoldingCmd(brain_, GamePiece.CORAL)) ;

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
