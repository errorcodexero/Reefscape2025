package frc.robot.commands.robot.placecoral;

import org.xerosw.util.XeroSequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class PlaceCoralTwoStepOne extends Command {

    private ManipulatorSubsystem m_ ;
    private XeroSequence sequence_ ;

    public PlaceCoralTwoStepOne(ManipulatorSubsystem m) {
        setName("PlaceCoralBeforeCmd") ;
        m_ = m ;
    }

    @Override
    public void initialize() {
        sequence_ = new XeroSequence() ;
        sequence_.addCommands(new GoToCmd(m_, ManipulatorConstants.Positions.kStowedHeight, ManipulatorConstants.Positions.kStowedAngle)) ;
    }
}
