package frc.robot.commands.robot;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ReefLevel;
import frc.robot.RobotContainer;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.DepositAlgaeCmd;
import frc.robot.subsystems.grabber.commands.DepositCoralCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmd;
import frc.robot.subsystems.manipulator.commands.GoToCmdDirect;

public class EjectCmd extends XeroSequenceCmd {
    private BrainSubsystem brain_ ;
    private ManipulatorSubsystem manipulator_;
    private GrabberSubsystem grabber_;

    public EjectCmd(BrainSubsystem brain, ManipulatorSubsystem manipulator, GrabberSubsystem grabber) {
        super("EjectCmd") ;
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
        super.initialize();
        brain_.lock() ;
        brain_.clearRobotActions() ;
    }

    @Override
    public void initSequence(SequentialCommandGroup seq) {
        if (brain_.gp() == GamePiece.CORAL) {
            seq.addCommands(
                new DepositCoralCmd(grabber_, ReefLevel.L4),
                new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)) ;
        }
        else {
            //
            // We are up with algae, just eject, but stay up until eject is hit twice
            //
            seq.addCommands(
                new DepositAlgaeCmd(grabber_),
                new GoToCmdDirect(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, manipulator_.getArmPosition()),
                new GoToCmdDirect(manipulator_, ManipulatorConstants.Elevator.Positions.kStow, ManipulatorConstants.Arm.Positions.kStow)
            ) ;
        }
        
        seq.addCommands(RobotContainer.getInstance().gamepad().setLockCommand(false));
        seq.addCommands(new SetHoldingCmd(brain_, GamePiece.NONE));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean ret = super.isFinished() ;
        if (ret) {
            brain_.unlock() ;
        }

        return ret;
    }
}
