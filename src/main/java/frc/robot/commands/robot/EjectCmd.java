package frc.robot.commands.robot;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositAlgaeCmd;
import frc.robot.subsystems.grabber.commands.DepositCoralCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class EjectCmd extends Command {
    private XeroSequence sequence_;
    private BrainSubsystem brain_ ;
    private ManipulatorSubsystem manipulator_;
    private GrabberSubsystem grabber_;

    public EjectCmd(BrainSubsystem brain, ManipulatorSubsystem manipulator, GrabberSubsystem grabber) {
        brain_ = brain ;
        manipulator_ = manipulator;
        grabber_ = grabber;
    }

    // COMMANDS NEEDED:
    // GoToCmd
    // WaitForCoral

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        brain_.lock() ;
        sequence_ = new XeroSequence();

        if (brain_.gp() == GamePiece.ALGAE_HIGH) {
            sequence_.addCommands(
                new DepositAlgaeCmd(grabber_),
                new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectL3, manipulator_.getArmPosition(), true),
                new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)) ;
        }
        else {
            sequence_.addCommands(
                new DepositCoralCmd(grabber_),
                new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)) ;            
        }
        sequence_.addCommands(RobotContainer.getInstance().gamepad().setLockCommand(false));
        sequence_.addCommands(new SetHoldingCmd(brain_, GamePiece.NONE));

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
        boolean ret = false ;

        if (sequence_.isComplete()) {
            brain_.unlock() ;
            ret = true ;
        }

        return ret;
    }
}
