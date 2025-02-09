package frc.robot.commands.robot.collectcoral;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.misc.RumbleGamepadCmd;
import frc.robot.commands.robot.ReportStateCmd;
import frc.robot.commands.robot.SetHoldingCmd;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.WaitForCoralCmd;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.RobotContainer;

public class CollectCoralCmd extends Command {

    private static final Angle ArmCollectAngle = Degrees.of(90.0) ;
    private static final Distance ElevatorCollectHeight = Meters.of(1.0) ;

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
        sequence_.addCommands(
            new ReportStateCmd(getName(), "goto"),
            new ParallelDeadlineGroup(
                new WaitForCoralCmd(grabber_),
                new GoToCmd(manipulator_, ElevatorCollectHeight, ArmCollectAngle)),
            new ReportStateCmd(getName(), "holding"),
            new SetHoldingCmd(brain_, RobotContainer.GamePiece.CORAL),
            new ReportStateCmd(getName(), "rumbling"),
            new RumbleGamepadCmd(Milliseconds.of(500)),
            new ReportStateCmd(getName(), "done")) ;

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
