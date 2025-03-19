package frc.robot.subsystems.grabber.commands;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ReefLevel;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmdDirect;

public class CollectAlgaeNewCmd extends SequentialCommandGroup {

    private final GrabberSubsystem grabber_;
    private final ManipulatorSubsystem mani_ ;

    public CollectAlgaeNewCmd(GrabberSubsystem grabber, ManipulatorSubsystem manip, ReefLevel level) {
        grabber_ = grabber;
        mani_ = manip ;

        Distance height = (level == ReefLevel.L1 || level == ReefLevel.L2) ? 
                                ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectNewPos2L2 : 
                                ManipulatorConstants.Elevator.Positions.kAlgaeReefCollectNewPos2L3 ;

        Angle angle = (level == ReefLevel.L1 || level == ReefLevel.L2) ? 
                                ManipulatorConstants.Arm.Positions.kAlgaeReefCollectNewPos2L2 : 
                                ManipulatorConstants.Arm.Positions.kAlgaeReefCollectNewPos2L3 ;

        addCommands(
            grabber.setVoltageCommand(Volts.of(-6.0)),
            logState("Waiting"),
            new GoToCmdDirect(manip, height, angle),
            Commands.waitUntil(this::hasAlgae),
            logState("Grabbed"),
            new WaitCommand(Milliseconds.of(500)),
            grabber.setVoltageCommand(Volts.of(GrabberConstants.Grabber.kHoldingVoltage))
        );
    }

    private boolean hasAlgae() {
        return !grabber_.algaeSensor();
    }

    private Command logState(String state) {
        return Commands.runOnce(() -> {
            Logger.recordOutput("Commands/AlgaeCollect/State", state);
        });
    }
}
