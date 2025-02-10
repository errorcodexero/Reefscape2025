package frc.robot.commands.robot.collectground;

import static edu.wpi.first.units.Units.Milliseconds;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.commands.robot.SetHoldingCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.CollectAlgaeCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CollectGroundAlgaeCmd extends Command {

    private XeroSequence sequence_ ;
    private BrainSubsystem b_ ;
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;

    public CollectGroundAlgaeCmd(BrainSubsystem b, ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("PlaceCoralCmd") ;

        b_ = b ;
        m_ = m ;
        g_ = g ;
    }

    @Override
    public void initialize() {
        sequence_ = new XeroSequence() ;
        sequence_.addCommands(
            new GoToCmd(m_, CollectGroundConstants.kElevatorImmdHeight, CollectGroundConstants.kArmStowAngle),
            new GoToCmd(m_, CollectGroundConstants.kElevatorImmdHeight, CollectGroundConstants.kArmCollectAngle),
            new GoToCmd(m_, CollectGroundConstants.kElevatorCollectHeight, CollectGroundConstants.kArmCollectAngle),
            new CollectAlgaeCmd(g_),
            new SetHoldingCmd(b_, GamePiece.ALGAE_LOW),
            new GoToCmd(m_, CollectGroundConstants.kElevatorStoreHeight, CollectGroundConstants.kArmStoreAngle),
            new RumbleGamepadCmd(Milliseconds.of(500))) ;
        sequence_.schedule();
    }

    @Override
    public boolean isFinished() {
        return sequence_.isComplete() ;
    }
}
