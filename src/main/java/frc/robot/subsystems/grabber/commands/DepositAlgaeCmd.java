package frc.robot.subsystems.grabber.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class DepositAlgaeCmd extends SequentialCommandGroup {

    public DepositAlgaeCmd(GrabberSubsystem grabber) {
        addCommands(
            grabber.setVoltageCommand(Volts.of(GrabberConstants.Grabber.kDepositVoltage)),
            Commands.waitTime(GrabberConstants.Grabber.DepositAlgae.delay),
            grabber.stopGrabberCommand()
        );
    }

}
