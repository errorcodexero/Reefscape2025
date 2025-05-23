package frc.robot.commands.robot.collectcoral;

import org.xerosw.util.XeroSequenceCmd;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.misc.StateCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.brain.SetHoldingCmd;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.WaitForCoralCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmd;
import frc.robot.subsystems.oi.OISubsystem;

public class CollectCoralCmd extends XeroSequenceCmd {
    private OISubsystem oi_ ;
    private ManipulatorSubsystem manipulator_;
    private GrabberSubsystem grabber_;
    private FunnelSubsystem funnel_;
    private BrainSubsystem brain_;
    private boolean moveman_ ;

    public CollectCoralCmd(BrainSubsystem brain, OISubsystem oi, ManipulatorSubsystem manipulator, FunnelSubsystem funnel, GrabberSubsystem grabber, boolean moveman) {
        super("CollectCoralCmd") ;
        manipulator_ = manipulator;
        grabber_ = grabber;
        brain_ = brain;
        funnel_ = funnel;
        oi_ = oi ;
        moveman_ = moveman ;
    }

    // COMMANDS NEEDED:
    // GoToCmd
    // WaitForCoral

    // Called when the command is initially scheduled.
    @Override
    public void initSequence(SequentialCommandGroup seq) {
        if (moveman_) {
            seq.addCommands(new StateCmd("collectcoral", "start")) ;
            seq.addCommands(
                new GoToCmd(manipulator_, ManipulatorConstants.Elevator.Positions.kCollect, ManipulatorConstants.Arm.Positions.kCollect)) ;
        }
        seq.addCommands(
            new StateCmd("collectcoral", "wait"),
            new WaitForCoralCmd(oi_, funnel_, grabber_),
            new SetHoldingCmd(brain_, GamePiece.CORAL)) ;
    }
}
