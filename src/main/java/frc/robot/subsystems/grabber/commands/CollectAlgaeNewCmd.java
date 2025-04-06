package frc.robot.subsystems.grabber.commands;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ReefLevel;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmdDirect;

public class CollectAlgaeNewCmd extends SequentialCommandGroup {

    public CollectAlgaeNewCmd(GrabberSubsystem grabber, ManipulatorSubsystem manip, ReefLevel level) {
        Distance height = (level == ReefLevel.L1 || level == ReefLevel.L2) ? 
                                ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectNewPos2L2 : 
                                ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectNewPos2L3 ;

        Angle angle = (level == ReefLevel.L1 || level == ReefLevel.L2) ? 
                                ManipulatorConstants.Arm.Positions.kAlgaeReefCollectNewPos2L2 : 
                                ManipulatorConstants.Arm.Positions.kAlgaeReefCollectNewPos2L3 ;

        addCommands(
            grabber.setVoltageCommand(Volts.of(-6.0)),
            new GoToCmdDirect(manip, height, angle),
            new WaitCommand(Milliseconds.of(250)),
            grabber.setVoltageCommand(Volts.of(GrabberConstants.Grabber.kHoldingVoltage))
        );
    }
}
