package frc.robot.commands.gps;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CollectReefAlgaeBeforeCmd extends SequentialCommandGroup {
    public CollectReefAlgaeBeforeCmd(ManipulatorSubsystem m, int level) {
        setName("PlaceCoralCmd") ;
        addCommands(new GoToCmd(m, CommandPositions.Place.ElevatorHeight[level], CommandPositions.Place.ArmAngle[level])) ;
    }
}
