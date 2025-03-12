package frc.robot.subsystems.grabber.commands;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class CollectAlgaeCmd extends SequentialCommandGroup {

    private final GrabberSubsystem grabber_;

    public CollectAlgaeCmd(GrabberSubsystem grabber) {
        grabber_ = grabber;

        addCommands(
            grabber.setVoltageCommand(Volts.of(12.0)),
            logState("Waiting"),
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
