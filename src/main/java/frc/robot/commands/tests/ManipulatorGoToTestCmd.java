package frc.robot.commands.tests;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ManipulatorGoToTestCmd extends SequentialCommandGroup {
    public ManipulatorGoToTestCmd(ManipulatorSubsystem m) {
        addCommands(
            new GoToCmd(m, Centimeters.of(60), Degrees.of(91.0)),
            new WaitCommand(Seconds.of(1.0)),
            new GoToCmd(m, Centimeters.of(60), Degrees.of(0.0)),
            new WaitCommand(Seconds.of(1.0)),
            new GoToCmd(m, Centimeters.of(60), Degrees.of(91.0)),
            new WaitCommand(Seconds.of(1.0)),
            new GoToCmd(m, Centimeters.of(0), Degrees.of(0.0)),
            new WaitCommand(Seconds.of(1.0)),
            new GoToCmd(m, Centimeters.of(0.0), Degrees.of(91.0)),
            new WaitCommand(Seconds.of(1.0)),
            new GoToCmd(m, Centimeters.of(0), Degrees.of(0.0))) ;
    }
}
