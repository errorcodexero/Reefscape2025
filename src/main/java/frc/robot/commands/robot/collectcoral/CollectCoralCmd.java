package frc.robot.commands.robot.collectcoral;

import static edu.wpi.first.units.Units.Milliseconds;

import org.xerosw.util.XeroSequence;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.commands.robot.SetHoldingCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.WaitForCoralCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CollectCoralCmd extends Command {
    private XeroSequence sequence_ ;
    private BrainSubsystem brain_ ;
    private ManipulatorSubsystem manipulator_ ;
    private GrabberSubsystem grabber_ ;

    public CollectCoralCmd(BrainSubsystem b, ManipulatorSubsystem m, GrabberSubsystem g) {
        setName("CollectCoralCmd") ;
        brain_ = b ;
        manipulator_ = m ;
        grabber_ = g ;
    }

    @Override
    public void initialize() {
        sequence_ = new XeroSequence() ;
        sequence_.addCommands(
            new ParallelDeadlineGroup(
                new WaitForCoralCmd(grabber_),
                new GoToCmd(manipulator_, CollectCoralConstants.kElevatorCollectHeight, CollectCoralConstants.kArmCollectAngle)),
            new SetHoldingCmd(brain_, GamePiece.CORAL),
            new RumbleGamepadCmd(Milliseconds.of(500))) ;

        sequence_.schedule() ;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        sequence_.cancel() ;
    }

    @Override
    public boolean isFinished() {
        return sequence_.isComplete() ;
    }
}
