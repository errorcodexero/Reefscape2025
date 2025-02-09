package frc.robot.commands.robot.collectreef;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CollectReefAlgaeBeforeCmd extends Command {
    private XeroSequence sequence_ ;
    private BrainSubsystem b_ ;
    private ManipulatorSubsystem m_ ;

    public CollectReefAlgaeBeforeCmd(BrainSubsystem b, ManipulatorSubsystem m) {
        setName("PlaceCoralCmd") ;

        b_ = b ;
        m_ = m ;
    }

    @Override
    public void initialize() {
        sequence_.addCommands(
            new GoToCmd(m_, CollectReefAlgaeConstants.Collect.ElevatorHeight[b_.level()], CollectReefAlgaeConstants.Collect.ArmAngle[b_.level()])) ;
    }
}
