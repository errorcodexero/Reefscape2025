package frc.robot.commands.robot.placecoral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class PlaceCoralTwoStepOne extends SequentialCommandGroup {

    public PlaceCoralTwoStepOne(ManipulatorSubsystem m, int level) {
        setName("PlaceCoralBeforeCmd") ;
    }
}
