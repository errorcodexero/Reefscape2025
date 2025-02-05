package frc.robot.commands.gps;

import frc.robot.subsystems.manipulator.ManipulatorGotoCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class PlaceCoralBeforeCmd extends PlaceCoralCmd {

    public PlaceCoralBeforeCmd(ManipulatorSubsystem m, int level) {
        setName("PlaceCoralCmd") ;
        addCommands(new ManipulatorGotoCmd(m, ElevatorPlaceHeight[level], ArmPlaceAngle[level])) ;
    }
}
