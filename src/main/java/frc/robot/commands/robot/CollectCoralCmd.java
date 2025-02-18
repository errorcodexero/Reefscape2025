package frc.robot.commands.robot;

import org.xerosw.util.XeroSequenceCmd;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.WaitForCoralCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CollectCoralCmd extends XeroSequenceCmd {
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
    public void initSequence(SequentialCommandGroup seq) {
        seq.addCommands(
            new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kCollect, ManipulatorConstants.Arm.Positions.kCollect),
            new WaitForCoralCmd(grabber_),
            new SetHoldingCmd(brain_, GamePiece.CORAL)) ;
    }
}
