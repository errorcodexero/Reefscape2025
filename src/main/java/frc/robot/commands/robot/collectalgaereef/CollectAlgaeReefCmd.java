package frc.robot.commands.robot.collectalgaereef;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReefLevel;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.CollectAlgaeCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CollectAlgaeReefCmd extends Command {
    private XeroSequence sequence_;
    private BrainSubsystem brain_ ;
    private ManipulatorSubsystem manipulator_;
    private GrabberSubsystem grabber_;
    private ReefLevel height_ ;

    public CollectAlgaeReefCmd(BrainSubsystem brain, ManipulatorSubsystem manipulator, GrabberSubsystem grabber, ReefLevel height) {
        brain_ = brain ;
        manipulator_ = manipulator;
        grabber_ = grabber;
        height_ = height ;
    }

    // COMMANDS NEEDED:
    // GoToCmd
    // WaitForCoral

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Angle angle ;
        Distance height ;
        
        if (height_ == ReefLevel.AskBrain) {
            height_ = brain_.algaeLevel() ;
        }

        if (height_ == ReefLevel.L2) {
            angle = ManipulatorConstants.Arm.Positions.kAlgaeReefCollectL2 ;
            height = ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL2 ;
        } else if (height_ == ReefLevel.L3) {
            angle = ManipulatorConstants.Arm.Positions.kAlgaeReefCollectL3 ;
            height = ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL3 ;
        }
        else {
            //
            // Bad height value, so we just return and have an empty sequence which
            // does nothing.
            //
            return ;
        }

        sequence_ = new XeroSequence();
        sequence_.addCommands(
            new GoToCmd(manipulator_, height, angle),
            new CollectAlgaeCmd(grabber_),
            new SetHoldingCmd(brain_, GamePiece.ALGAE_HIGH),
            new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kAlgaeReefHold, ManipulatorConstants.Arm.Positions.kAlgaeReefHold)) ;

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
