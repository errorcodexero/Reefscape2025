package frc.robot.commands.robot.collectalgaereef;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;
import java.util.concurrent.locks.Condition;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ReefLevel;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmdDirect;

public class CollectAlgaeReefGotoCmd extends XeroSequenceCmd {

    private BrainSubsystem b_ ;
    private ManipulatorSubsystem m_ ;
    private ReefLevel level_ ;

    public CollectAlgaeReefGotoCmd(BrainSubsystem b, ManipulatorSubsystem m, ReefLevel l) {
        super("CollectAlgaeReefGotoCmd") ;

        m_ = m ;
        level_ = l ;
        b_ = b ;
    }

    @Override
    public void initSequence(SequentialCommandGroup seq) {
        ReefLevel level = level_ ;

        Angle angle ;
        Distance height ;
        
        if (level_ == ReefLevel.AskBrain) {
            level = b_.algaeLevel() ;
        }

        if (level == ReefLevel.L2 || level == ReefLevel.L1) {
            angle = ManipulatorConstants.Arm.Positions.kAlgaeReefCollectNewL2 ;
            height = ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectNewL2 ;
        } else if (level == ReefLevel.L3 || level == ReefLevel.L4) {
            angle = ManipulatorConstants.Arm.Positions.kAlgaeReefCollectNewL3 ;
            height = ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectNewL3 ;
        }
        else {
            //
            // Bad height value, so we just return and have an empty sequence which
            // does nothing.
            //
            return ;
        }

        seq.addCommands(
            new ConditionalCommand(
                Commands.none(),
                new GoToCmdDirect(m_, ManipulatorConstants.Elevator.Positions.kStow, angle),
                this::angleOK)) ;
        seq.addCommands(new GoToCmdDirect(m_, height, angle)) ;
    }

    private boolean angleOK() {
        return m_.getArmPosition().gte(Degrees.of(100.0)) ;
    }
}
